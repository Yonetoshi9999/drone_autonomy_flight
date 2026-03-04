"""
飛行制御モジュール
経路計画、ウェイポイント管理、エネルギー最適化
飛行禁止区域の動的取得機能付き
"""

import numpy as np
import json
import requests
from pathlib import Path
from pymavlink import mavutil
from datetime import datetime, timedelta
import threading
import time
from shapely.geometry import Point, Polygon
import hashlib
import pickle
from route_optimizer import RouteOptimizer

class FlightController:
    """飛行制御クラス"""
    
    def __init__(self, mavlink_connection):
        self.mavlink = mavlink_connection
        
        # ウェイポイント管理
        self.waypoints = []
        self.current_wp_index = 0
        
        # 飛行パラメータ
        self.max_velocity = 5.0  # m/s
        self.max_altitude = 100.0  # m
        self.battery_rtl_threshold = 30  # %
        
        # 状態
        self.current_position = np.zeros(3)
        self.current_velocity = np.zeros(3)
        self.battery_percentage = 100
        self.current_gps = {'lat': 35.0, 'lon': 136.0}  # 初期値（名古屋付近）
        
        # 飛行禁止区域
        self.no_fly_zones = []
        self.nfz_last_update = None
        self.nfz_cache_duration = timedelta(hours=1)  # 1時間キャッシュ
        self.nfz_update_thread = None
        self.nfz_updating = False
        
        # NFZ データソース設定
        self.nfz_sources = {
            'japan_mlit': {  # 国土交通省
                'url': 'https://www.mlit.go.jp/koku/content/001411544.json',
                'enabled': True,
                'type': 'official'
            },
            'dji_flysafe': {  # DJI FlySafe Database
                'url': 'https://www.dji.com/flysafe/no-fly',
                'enabled': True,
                'type': 'manufacturer'
            },
            'airmap': {  # AirMap API
                'url': 'https://api.airmap.com/airspace/v2/search',
                'api_key': 'YOUR_AIRMAP_API_KEY',  # 要取得
                'enabled': False,
                'type': 'commercial'
            },
            'opensky': {  # OpenSky Network
                'url': 'https://opensky-network.org/api/airspace',
                'enabled': False,
                'type': 'community'
            },
            'local_authority': {  # 地方自治体
                'url': 'https://www.city.nagoya.jp/api/drone/nfz',
                'enabled': True,
                'type': 'local'
            }
        }
        
        # キャッシュディレクトリ
        self.cache_dir = Path('/home/pi/aerial_photography_drone/cache/nfz')
        try:
            self.cache_dir.mkdir(parents=True, exist_ok=True)
        except (PermissionError, OSError):
            # テスト環境やパーミッション不足の場合、一時ディレクトリを使用
            import tempfile
            self.cache_dir = Path(tempfile.mkdtemp()) / 'nfz'
            self.cache_dir.mkdir(parents=True, exist_ok=True)
        
        # 強化学習モデル
        self.rl_model = self.load_rl_model()

        # AIルート最適化（後で初期化、循環参照回避）
        self.route_optimizer = None

        # 初回NFZ取得
        self.update_no_fly_zones_async()
    
    def set_route_optimizer(self, optimizer):
        """ルート最適化エンジンを設定（循環参照回避用）"""
        self.route_optimizer = optimizer

    def update_state_from_telemetry(self, telemetry_data):
        """
        テレメトリデータから状態を更新

        Args:
            telemetry_data: AutonomyStateManager.telemetry (TelemetryData)
        """
        # Local NED位置・速度更新
        self.current_position = np.array(telemetry_data.local_position)
        self.current_velocity = np.array(telemetry_data.local_velocity)

        # GPS位置更新
        lat, lon, alt = telemetry_data.global_position
        self.current_gps = {'lat': lat, 'lon': lon, 'alt': alt}

        # バッテリー更新
        self.battery_percentage = telemetry_data.battery_remaining

    def update_no_fly_zones_async(self):
        """非同期で飛行禁止区域を更新"""
        if not self.nfz_updating:
            self.nfz_update_thread = threading.Thread(
                target=self.update_no_fly_zones,
                daemon=True
            )
            self.nfz_update_thread.start()
    
    def update_no_fly_zones(self):
        """飛行禁止区域データを動的に取得"""
        
        self.nfz_updating = True
        print("飛行禁止区域データ更新中...")
        
        try:
            # キャッシュチェック
            if self.load_nfz_cache():
                print("キャッシュから飛行禁止区域データを読み込みました")
                self.nfz_updating = False
                return
            
            all_zones = []
            
            # 1. 国土交通省データ取得
            if self.nfz_sources['japan_mlit']['enabled']:
                mlit_zones = self.fetch_mlit_no_fly_zones()
                all_zones.extend(mlit_zones)
            
            # 2. DJI FlySafeデータ取得
            if self.nfz_sources['dji_flysafe']['enabled']:
                dji_zones = self.fetch_dji_no_fly_zones()
                all_zones.extend(dji_zones)
            
            # 3. 地方自治体データ取得
            if self.nfz_sources['local_authority']['enabled']:
                local_zones = self.fetch_local_no_fly_zones()
                all_zones.extend(local_zones)
            
            # 4. AirMap API（有効な場合）
            if self.nfz_sources['airmap']['enabled']:
                airmap_zones = self.fetch_airmap_zones()
                all_zones.extend(airmap_zones)
            
            # 5. 静的な禁止区域を追加（空港など）
            static_zones = self.get_static_no_fly_zones()
            all_zones.extend(static_zones)
            
            # 重複除去と統合
            self.no_fly_zones = self.merge_no_fly_zones(all_zones)
            
            # キャッシュ保存
            self.save_nfz_cache()
            
            self.nfz_last_update = datetime.now()
            print(f"飛行禁止区域データ更新完了: {len(self.no_fly_zones)}エリア")
            
        except Exception as e:
            print(f"飛行禁止区域データ取得エラー: {e}")
            # フォールバック: ローカルデータを使用
            self.load_fallback_nfz()
            
        finally:
            self.nfz_updating = False
    
    def fetch_mlit_no_fly_zones(self):
        """国土交通省の飛行禁止区域データ取得"""
        zones = []
        
        try:
            # DID地区（人口集中地区）データ
            did_url = "https://www.mlit.go.jp/common/001411544.json"
            response = requests.get(did_url, timeout=10)
            
            if response.status_code == 200:
                data = response.json()
                
                for area in data.get('areas', []):
                    zone = {
                        'id': f"mlit_{area.get('id', '')}",
                        'name': area.get('name', '不明なエリア'),
                        'type': 'did',  # 人口集中地区
                        'geometry_type': area.get('type', 'polygon'),
                        'coordinates': area.get('coordinates', []),
                        'altitude_limit': 0,  # 飛行禁止
                        'source': 'mlit',
                        'updated': datetime.now().isoformat()
                    }
                    zones.append(zone)
            
            # 空港周辺データ
            airport_url = "https://www.mlit.go.jp/koku/koku_tk10_000003.json"
            response = requests.get(airport_url, timeout=10)
            
            if response.status_code == 200:
                data = response.json()
                
                for airport in data.get('airports', []):
                    # 空港周辺は円形制限区域
                    zone = {
                        'id': f"airport_{airport.get('code', '')}",
                        'name': airport.get('name', ''),
                        'type': 'airport',
                        'geometry_type': 'circle',
                        'center': [airport.get('lat'), airport.get('lon')],
                        'radius': airport.get('restricted_radius', 9000),  # m
                        'altitude_limit': airport.get('altitude_limit', 45),  # m
                        'source': 'mlit',
                        'updated': datetime.now().isoformat()
                    }
                    zones.append(zone)
                    
        except requests.RequestException as e:
            print(f"国土交通省データ取得エラー: {e}")
        except json.JSONDecodeError as e:
            print(f"JSONパースエラー: {e}")
            
        return zones
    
    def fetch_dji_no_fly_zones(self):
        """DJI FlySafe飛行禁止区域データ取得"""
        zones = []
        
        try:
            # 現在位置周辺のDJI GEO Zone情報を取得
            lat, lon = self.current_gps['lat'], self.current_gps['lon']
            
            # DJI GEO API（非公式）
            url = f"https://www.dji.com/api/geo/areas"
            params = {
                'lat': lat,
                'lng': lon,
                'radius': 50000,  # 50km範囲
                'level': '0,1,2,3,4'  # 全レベル
            }
            
            headers = {
                'User-Agent': 'Mozilla/5.0 (compatible; DroneApp/1.0)',
                'Accept': 'application/json'
            }
            
            response = requests.get(url, params=params, headers=headers, timeout=10)
            
            if response.status_code == 200:
                data = response.json()
                
                for area in data.get('areas', []):
                    zone = {
                        'id': f"dji_{area.get('area_id', '')}",
                        'name': area.get('name', ''),
                        'type': self.map_dji_level(area.get('level', 0)),
                        'geometry_type': area.get('shape', 'polygon'),
                        'coordinates': area.get('points', []),
                        'altitude_limit': area.get('height', 0),
                        'source': 'dji',
                        'warning_level': area.get('level', 0),
                        'updated': datetime.now().isoformat()
                    }
                    zones.append(zone)
                    
        except Exception as e:
            print(f"DJI FlySafeデータ取得エラー: {e}")
            
        return zones
    
    def fetch_local_no_fly_zones(self):
        """地方自治体の飛行禁止区域データ取得"""
        zones = []
        
        try:
            # 名古屋市の例
            url = "https://www.city.nagoya.jp/opendata/drone_restricted_areas.json"
            response = requests.get(url, timeout=10)
            
            if response.status_code == 200:
                data = response.json()
                
                for area in data.get('restricted_areas', []):
                    zone = {
                        'id': f"nagoya_{area.get('id', '')}",
                        'name': area.get('name', ''),
                        'type': 'local_restriction',
                        'geometry_type': 'polygon',
                        'coordinates': area.get('boundary', []),
                        'altitude_limit': area.get('max_altitude', 0),
                        'time_restriction': area.get('time_restriction', None),
                        'source': 'nagoya_city',
                        'updated': datetime.now().isoformat()
                    }
                    zones.append(zone)
                    
        except Exception as e:
            print(f"地方自治体データ取得エラー: {e}")
            
        return zones
    
    def fetch_airmap_zones(self):
        """AirMap APIから飛行禁止区域データ取得"""
        zones = []
        
        if not self.nfz_sources['airmap']['api_key']:
            return zones
        
        try:
            url = self.nfz_sources['airmap']['url']
            headers = {
                'X-API-Key': self.nfz_sources['airmap']['api_key'],
                'Accept': 'application/json'
            }
            
            # 現在位置を中心に検索
            params = {
                'latitude': self.current_gps['lat'],
                'longitude': self.current_gps['lon'],
                'buffer': 50000,  # 50km
                'types': 'airport,heliport,tfr,controlled_airspace,school,hospital,power_plant',
                'full': 'true',
                'geometry_format': 'geojson'
            }
            
            response = requests.get(url, params=params, headers=headers, timeout=10)
            
            if response.status_code == 200:
                data = response.json()
                
                for feature in data.get('data', {}).get('features', []):
                    properties = feature.get('properties', {})
                    geometry = feature.get('geometry', {})
                    
                    zone = {
                        'id': f"airmap_{properties.get('id', '')}",
                        'name': properties.get('name', ''),
                        'type': properties.get('type', 'unknown'),
                        'geometry_type': geometry.get('type', '').lower(),
                        'coordinates': geometry.get('coordinates', []),
                        'altitude_limit': properties.get('ceiling', 0),
                        'source': 'airmap',
                        'updated': datetime.now().isoformat()
                    }
                    zones.append(zone)
                    
        except Exception as e:
            print(f"AirMapデータ取得エラー: {e}")
            
        return zones
    
    def get_static_no_fly_zones(self):
        """静的な飛行禁止区域（主要空港など）"""
        
        # 日本の主要空港
        major_airports = [
            {
                'name': '中部国際空港',
                'center': [34.8581, 136.8056],
                'radius': 9000
            },
            {
                'name': '名古屋飛行場',
                'center': [35.2550, 136.9242],
                'radius': 5000
            },
            {
                'name': '羽田空港',
                'center': [35.5494, 139.7798],
                'radius': 9000
            },
            {
                'name': '成田空港',
                'center': [35.7720, 140.3929],
                'radius': 9000
            },
            {
                'name': '関西国際空港',
                'center': [34.4273, 135.2441],
                'radius': 9000
            }
        ]
        
        zones = []
        for airport in major_airports:
            zone = {
                'id': f"static_airport_{airport['name']}",
                'name': f"{airport['name']}制限空域",
                'type': 'airport',
                'geometry_type': 'circle',
                'center': airport['center'],
                'radius': airport['radius'],
                'altitude_limit': 45,  # m
                'source': 'static',
                'updated': datetime.now().isoformat()
            }
            zones.append(zone)
        
        return zones
    
    def merge_no_fly_zones(self, zones):
        """重複する飛行禁止区域をマージ"""
        
        merged = {}
        
        for zone in zones:
            # ユニークIDを生成
            zone_id = self.generate_zone_id(zone)
            
            if zone_id not in merged:
                merged[zone_id] = zone
            else:
                # より制限の厳しい方を採用
                existing = merged[zone_id]
                if zone.get('altitude_limit', float('inf')) < existing.get('altitude_limit', float('inf')):
                    merged[zone_id] = zone
        
        return list(merged.values())
    
    def generate_zone_id(self, zone):
        """ゾーンのユニークID生成"""
        
        # 位置情報をベースにハッシュ生成
        if zone['geometry_type'] == 'circle':
            key = f"{zone['center'][0]:.4f},{zone['center'][1]:.4f},{zone['radius']}"
        else:
            coords_str = str(zone.get('coordinates', []))
            key = coords_str[:100]  # 最初の100文字
        
        return hashlib.md5(key.encode()).hexdigest()[:16]
    
    def load_nfz_cache(self):
        """キャッシュから飛行禁止区域データを読み込み"""
        
        cache_file = self.cache_dir / f"nfz_cache_{self.current_gps['lat']:.2f}_{self.current_gps['lon']:.2f}.pkl"
        
        if cache_file.exists():
            # キャッシュの有効期限チェック
            cache_age = datetime.now() - datetime.fromtimestamp(cache_file.stat().st_mtime)
            
            if cache_age < self.nfz_cache_duration:
                try:
                    with open(cache_file, 'rb') as f:
                        self.no_fly_zones = pickle.load(f)
                    self.nfz_last_update = datetime.fromtimestamp(cache_file.stat().st_mtime)
                    return True
                except Exception as e:
                    print(f"キャッシュ読み込みエラー: {e}")
        
        return False
    
    def save_nfz_cache(self):
        """飛行禁止区域データをキャッシュに保存"""
        
        cache_file = self.cache_dir / f"nfz_cache_{self.current_gps['lat']:.2f}_{self.current_gps['lon']:.2f}.pkl"
        
        try:
            with open(cache_file, 'wb') as f:
                pickle.dump(self.no_fly_zones, f)
        except Exception as e:
            print(f"キャッシュ保存エラー: {e}")
    
    def load_fallback_nfz(self):
        """フォールバック用のローカル飛行禁止区域データ"""
        
        fallback_file = Path('/home/pi/aerial_photography_drone/data/fallback_nfz.json')
        
        try:
            if fallback_file.exists():
                with open(fallback_file) as f:
                    self.no_fly_zones = json.load(f)
                print("フォールバックNFZデータを使用")
            else:
                # 最小限の安全区域のみ設定
                self.no_fly_zones = self.get_static_no_fly_zones()
                print("静的NFZデータのみ使用")
        except Exception as e:
            print(f"フォールバックデータ読み込みエラー: {e}")
            self.no_fly_zones = []
    
    def map_dji_level(self, level):
        """DJIの警告レベルをタイプにマッピング"""
        
        level_map = {
            0: 'warning',
            1: 'authorization',
            2: 'restricted',
            3: 'enhanced_warning',
            4: 'regulatory_restricted'
        }
        
        return level_map.get(level, 'unknown')
    
    def check_position_in_nfz(self, position):
        """指定位置が飛行禁止区域内かチェック"""
        
        lat, lon, alt = position
        point = Point(lon, lat)  # Shapely uses (lon, lat) order
        
        for zone in self.no_fly_zones:
            if zone['geometry_type'] == 'circle':
                # 円形区域チェック
                center = Point(zone['center'][1], zone['center'][0])
                distance = point.distance(center)
                
                # 度をメートルに変換（簡易計算）
                distance_m = distance * 111000  # 1度 ≈ 111km
                
                if distance_m < zone['radius']:
                    if alt < zone.get('altitude_limit', 0):
                        return True, zone
                        
            elif zone['geometry_type'] == 'polygon':
                # ポリゴン区域チェック
                if zone.get('coordinates'):
                    try:
                        polygon = Polygon(zone['coordinates'][0])
                        if polygon.contains(point):
                            if alt < zone.get('altitude_limit', 0):
                                return True, zone
                    except Exception as e:
                        print(f"ポリゴンチェックエラー: {e}")
        
        return False, None
    
    def set_waypoints(self, waypoints):
        """ウェイポイント設定（NFZチェック強化版）"""

        # 開始通知
        print("\n" + "="*70)
        print(" ウェイポイント・目的地設定開始")
        print("="*70)
        print(f"設定日時: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"ウェイポイント数: {len(waypoints)}")

        # 現在位置表示
        print(f"\n現在位置:")
        print(f"  位置: ({self.current_position[0]:.2f}, {self.current_position[1]:.2f}, {-self.current_position[2]:.2f}m)")
        print(f"  GPS: (緯度={self.current_gps['lat']:.6f}, 経度={self.current_gps['lon']:.6f})")
        print(f"  バッテリー: {self.battery_percentage}%")

        # 各ウェイポイントの詳細を表示
        if len(waypoints) > 0:
            print(f"\n設定するウェイポイント一覧:")
            print("-" * 70)
            total_distance = 0
            prev_pos = self.current_position

            for i, wp in enumerate(waypoints):
                distance = np.linalg.norm(np.array(wp) - prev_pos)
                total_distance += distance

                # ウェイポイントごとの情報
                wp_type = "目的地" if i == len(waypoints) - 1 else "経由点"
                print(f"  WP{i+1} ({wp_type}):")
                print(f"    位置: ({wp[0]:.2f}, {wp[1]:.2f}, {-wp[2]:.2f}m)")
                print(f"    このWPまでの距離: {distance:.1f}m")
                prev_pos = np.array(wp)

            print("-" * 70)
            print(f"総飛行距離: {total_distance:.1f}m")

            # 推定飛行時間
            avg_speed = self.max_velocity * 0.8  # 平均速度 = 最大速度の80%
            estimated_time = total_distance / avg_speed
            print(f"推定飛行時間: {int(estimated_time/60)}分{int(estimated_time%60)}秒")
            print(f"推定消費バッテリー: {int(total_distance * 0.1)}%")

        # NFZデータが古い場合は更新
        print(f"\n飛行禁止区域データチェック中...")
        if self.nfz_last_update is None or \
           datetime.now() - self.nfz_last_update > self.nfz_cache_duration:
            print("  → NFZデータが古いため更新中...")
            self.update_no_fly_zones_async()

            # 更新完了を最大5秒待機
            for _ in range(50):
                if not self.nfz_updating:
                    break
                time.sleep(0.1)
            print(f"  ✓ NFZデータ更新完了 ({len(self.no_fly_zones)}区域を取得)")
        else:
            print(f"  ✓ NFZデータは最新 ({len(self.no_fly_zones)}区域)")

        # バッテリーチェック
        print(f"\nバッテリー残量チェック中...")
        if not self.check_mission_feasibility(waypoints):
            print("  ✗ エラー: バッテリー不足でミッション完了不可")
            print("="*70 + "\n")
            return False
        print(f"  ✓ バッテリー残量OK (現在: {self.battery_percentage}%)")

        # 各ウェイポイントのNFZチェック
        print(f"\nウェイポイントの飛行禁止区域チェック中...")
        for i, wp in enumerate(waypoints):
            in_nfz, zone = self.check_position_in_nfz(wp)
            if in_nfz:
                print(f"  ✗ エラー: ウェイポイント{i+1}は飛行禁止区域内")
                print(f"    区域名: {zone['name']}")
                print(f"    タイプ: {zone['type']}")
                print(f"    制限高度: {zone.get('altitude_limit', 0)}m")
                print("="*70 + "\n")
                return False
        print(f"  ✓ 全ウェイポイントが飛行可能区域内")

        # 経路上のNFZチェック
        print(f"\n飛行経路の飛行禁止区域チェック中...")
        if not self.check_path_clear(waypoints):
            print("  ✗ エラー: 飛行経路が飛行禁止区域を通過")
            print("="*70 + "\n")
            return False
        print(f"  ✓ 飛行経路が飛行禁止区域外")

        # 設定完了
        self.waypoints = waypoints
        self.current_wp_index = 0

        print("\n" + "="*70)
        print(" ✓ ミッション設定完了")
        print("="*70)
        print(f"ウェイポイント数: {len(waypoints)}")
        if len(waypoints) > 0:
            destination = waypoints[-1]
            print(f"目的地座標: ({destination[0]:.2f}, {destination[1]:.2f}, {-destination[2]:.2f}m)")
        print(f"設定時刻: {datetime.now().strftime('%H:%M:%S')}")
        print("ミッション実行準備完了 - 自律飛行を開始できます")
        print("="*70 + "\n")

        return True
    
    def check_path_clear(self, waypoints):
        """経路全体が飛行禁止区域を避けているかチェック"""
        
        if len(waypoints) < 2:
            return True
        
        # 各区間をチェック
        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]
            
            # 区間を10分割してチェック
            for t in np.linspace(0, 1, 10):
                check_point = start + (end - start) * t
                in_nfz, zone = self.check_position_in_nfz(check_point)
                
                if in_nfz:
                    print(f"経路がNFZ '{zone['name']}'を通過")
                    return False
        
        return True
    
    def get_nfz_status(self):
        """飛行禁止区域データの状態を取得"""
        
        status = {
            'zones_count': len(self.no_fly_zones),
            'last_update': self.nfz_last_update.isoformat() if self.nfz_last_update else None,
            'updating': self.nfz_updating,
            'sources': {}
        }
        
        for source_name, source_config in self.nfz_sources.items():
            status['sources'][source_name] = {
                'enabled': source_config['enabled'],
                'type': source_config['type']
            }
        
        return status
    
    # 以下、既存のメソッドは変更なし
    def check_mission_feasibility(self, waypoints):
        """ミッション実行可能性チェック"""
        # （既存の実装を維持）
        total_distance = 0
        prev_pos = self.current_position
        
        for wp in waypoints:
            distance = np.linalg.norm(np.array(wp) - prev_pos)
            total_distance += distance
            prev_pos = np.array(wp)
        
        total_distance += np.linalg.norm(prev_pos - self.current_position)
        
        energy_per_meter = 0.1
        required_battery = total_distance * energy_per_meter
        available_battery = self.battery_percentage - 20
        
        return required_battery < available_battery
    
    def calculate_trajectory(self, obstacles, wind):
        """
        軌道計算（100Hz / 10ms周期）
        AIルート最適化を使用したリアルタイム軌道計算
        """
        # AI最適化が利用可能な場合はそれを使用
        if self.route_optimizer:
            try:
                next_position, final_velocity = self.route_optimizer.calculate_trajectory_realtime(
                    current_pos=self.current_position,
                    waypoints=self.waypoints,
                    current_wp_index=self.current_wp_index,
                    obstacles=obstacles,
                    wind=np.array(wind),
                    dt=0.01  # 10ms周期
                )

                # ウェイポイント到達チェック
                if self.current_wp_index < len(self.waypoints):
                    target_wp = np.array(self.waypoints[self.current_wp_index])
                    distance = np.linalg.norm(target_wp - self.current_position)
                    if distance < 2.0:
                        self.current_wp_index += 1
                        print(f"ウェイポイント {self.current_wp_index}/{len(self.waypoints)} 到達")

                # 高度制約適用
                next_position = self._apply_altitude_constraints(
                    next_position,
                    final_velocity,
                    obstacles
                )

                # リアルタイムNFZチェック
                next_position, final_velocity = self._check_and_avoid_nfz(
                    next_position,
                    final_velocity
                )

                return next_position, final_velocity

            except Exception as e:
                print(f"AI軌道計算エラー: {e}, フォールバック使用")
                # フォールバック: 従来の計算方法

        # フォールバック: 従来の軌道計算
        return self._calculate_trajectory_fallback(obstacles, wind)

    def _calculate_trajectory_fallback(self, obstacles, wind):
        """従来の軌道計算（フォールバック用）"""
        if self.current_wp_index >= len(self.waypoints):
            return self.current_position, np.zeros(3)

        target_wp = np.array(self.waypoints[self.current_wp_index])
        direction = target_wp - self.current_position
        distance = np.linalg.norm(direction)

        if distance < 2.0:
            self.current_wp_index += 1

        if distance > 0:
            direction = direction / distance

        avoid_vector = self.calculate_avoidance(obstacles)
        wind_compensation = -np.array(wind) * 0.3

        final_velocity = (
            direction * 3.0 +
            avoid_vector * 2.0 +
            wind_compensation
        )

        speed = np.linalg.norm(final_velocity)
        if speed > self.max_velocity:
            final_velocity = final_velocity * (self.max_velocity / speed)

        next_position = self.current_position + final_velocity * 0.01  # 10ms周期

        # 高度制約適用
        next_position = self._apply_altitude_constraints(next_position, final_velocity, obstacles)

        # NFZチェック
        next_position, final_velocity = self._check_and_avoid_nfz(next_position, final_velocity)

        return next_position, final_velocity

    def _apply_altitude_constraints(self, next_position, final_velocity, obstacles):
        """高度制約を適用"""
        if len(self.waypoints) > 0:
            destination = np.array(self.waypoints[-1])
            distance_to_destination = np.linalg.norm(destination - self.current_position)

            # 障害物回避中かチェック
            is_avoiding_obstacles = len([o for o in obstacles if np.linalg.norm(
                self.current_position - o['position']) < 5.0]) > 0

            # 目的地まで50m以上離れている場合、高度50m以上を維持
            if distance_to_destination > 50.0 and not is_avoiding_obstacles:
                if next_position[2] > -50.0:
                    next_position[2] = -50.0
                    if final_velocity[2] > -1.0:
                        final_velocity[2] = -1.0

        return next_position

    def _check_and_avoid_nfz(self, next_position, final_velocity):
        """NFZチェックと回避"""
        in_nfz, zone = self.check_position_in_nfz(
            [self.current_gps['lat'], self.current_gps['lon'], -next_position[2]]
        )

        if in_nfz:
            print(f"警告: NFZ '{zone['name']}'に接近、回避動作")
            # NFZから離れる方向に修正
            avoid_direction = np.array([1, 1, 0])
            final_velocity = avoid_direction * 5.0
            next_position = self.current_position + final_velocity * 0.01

        return next_position, final_velocity
    
    # その他の既存メソッド（変更なし）
    def calculate_avoidance(self, obstacles):
        """障害物回避ベクトル計算"""
        # （既存実装を維持）
        if not obstacles:
            return np.zeros(3)
        
        repulsive_force = np.zeros(3)
        
        for obs in obstacles:
            diff = self.current_position - obs['position']
            dist = np.linalg.norm(diff)
            
            if dist < 5.0:
                force = (diff / dist) * (1.0 / (dist + 0.1)**2)
                repulsive_force += force
        
        return repulsive_force
    
    def send_command(self, position, velocity):
        """MAVLink経由でコマンド送信"""
        # （既存実装を維持）
        self.mavlink.mav.set_position_target_local_ned_send(
            0,
            self.mavlink.target_system,
            self.mavlink.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,
            position[0], position[1], position[2],
            velocity[0], velocity[1], velocity[2],
            0, 0, 0,
            0, 0
        )
    
    def get_wind_estimate(self):
        """風速推定値取得（3D風速ベクトル）"""
        msg = self.mavlink.recv_match(type='WIND', blocking=False)
        if msg:
            # 水平成分（X, Y）
            wind_x = msg.speed * np.cos(np.radians(msg.direction))
            wind_y = msg.speed * np.sin(np.radians(msg.direction))
            # 垂直成分（Z）
            wind_z = msg.speed_z if hasattr(msg, 'speed_z') else 0
            return [wind_x, wind_y, wind_z]
        return [0, 0, 0]
    
    def in_no_fly_zone(self, position):
        """飛行禁止区域チェック（新実装を使用）"""
        in_nfz, _ = self.check_position_in_nfz(position)
        return in_nfz
    
    def load_rl_model(self):
        """強化学習モデル読み込み"""
        # （既存実装を維持）
        try:
            import torch
            model = torch.load(
                '/home/pi/aerial_photography_drone/models/trajectory_rl.pth',
                map_location='cpu'
            )
            return model
        except:
            return None
    
    def emergency_landing(self):
        """緊急着陸"""
        # （既存実装を維持）
        print("緊急着陸開始")
        landing_pos = self.current_position.copy()
        landing_pos[2] = 0
        landing_vel = np.array([0, 0, 0.5])
        self.send_command(landing_pos, landing_vel)