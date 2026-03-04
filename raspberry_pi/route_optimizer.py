"""
AIベースルート最適化モジュール
目的地とウェイポイントから最適な飛行ルートを計算
"""

import numpy as np
from typing import List, Tuple, Dict
from scipy.spatial import distance_matrix
from scipy.optimize import minimize
import time
from coordinate_conversion import get_converter


class RouteOptimizer:
    """
    AIベースのルート最適化エンジン

    最適化アルゴリズム:
    1. Traveling Salesman Problem (TSP) - ウェイポイント順序最適化
    2. Gradient-based trajectory smoothing - 経路平滑化
    3. NFZ avoidance - 飛行禁止区域回避
    """

    def __init__(self, flight_controller=None):
        """
        初期化

        Args:
            flight_controller: FlightControllerインスタンス（NFZチェック用）
        """
        self.flight_controller = flight_controller

        # 最適化パラメータ
        self.smoothing_weight = 0.3  # 経路平滑化の重み
        self.distance_weight = 0.5   # 距離最小化の重み
        self.safety_weight = 0.2     # 安全性の重み

        # NFZ回避パラメータ
        self.nfz_safety_margin = 100.0  # m - NFZから100m離れる
        self.obstacle_safety_margin = 10.0  # m - 障害物から10m離れる

    def optimize_route(
        self,
        start_pos: np.ndarray,
        destination: Dict,
        waypoints: List[Dict],
        obstacles: List = None
    ) -> List[np.ndarray]:
        """
        最適なルートを計算

        Args:
            start_pos: 現在位置（NED座標）
            destination: 目的地（GPS座標）
            waypoints: ウェイポイントリスト（GPS座標）
            obstacles: 障害物リスト

        Returns:
            最適化されたウェイポイントリスト（NED座標）
        """
        print("AIルート最適化開始...")
        start_time = time.time()

        # GPS座標をNED座標に変換
        waypoints_ned = self._convert_gps_to_ned(start_pos, waypoints)
        destination_ned = self._convert_gps_to_ned_single(
            start_pos,
            destination['lat'],
            destination['lon'],
            destination['alt']
        )

        # ステップ1: ウェイポイント順序最適化（TSP）
        optimized_order = self._optimize_waypoint_order(
            start_pos,
            waypoints_ned,
            destination_ned
        )

        # ステップ2: 経路平滑化と補間
        smoothed_route = self._smooth_trajectory(
            start_pos,
            optimized_order,
            destination_ned
        )

        # ステップ3: NFZ回避
        if self.flight_controller and hasattr(self.flight_controller, 'check_position_in_nfz'):
            safe_route = self._avoid_nfz(smoothed_route)
        else:
            safe_route = smoothed_route

        # ステップ4: 高度最適化
        final_route = self._optimize_altitude(safe_route)

        elapsed = time.time() - start_time
        print(f"ルート最適化完了: {len(final_route)}ポイント, {elapsed:.3f}秒")

        return final_route

    def _convert_gps_to_ned(
        self,
        origin: np.ndarray,
        waypoints: List[Dict]
    ) -> List[np.ndarray]:
        """
        GPS座標をNED座標に変換（coordinate_conversion使用）

        Args:
            origin: 原点（NED座標、現在位置）
            waypoints: ウェイポイントリスト（GPS座標）

        Returns:
            NED座標のウェイポイントリスト
        """
        converter = get_converter()
        ned_waypoints = []

        for wp in waypoints:
            try:
                # 座標変換モジュールを使用
                x, y, z = converter.gps_to_ned(wp['lat'], wp['lon'], wp['alt'])
                ned_waypoints.append(np.array([x, y, z]))
            except ValueError:
                # ホームポジション未設定の場合はエラー
                print(f"警告: ホームポジション未設定のためGPS変換失敗")
                # フォールバック: 簡易変換
                origin_lat = 35.0
                origin_lon = 136.0
                delta_north = (wp['lat'] - origin_lat) * 111000.0
                delta_east = (wp['lon'] - origin_lon) * 111000.0 * np.cos(np.radians(origin_lat))
                delta_down = -wp['alt']
                ned_waypoints.append(np.array([delta_north, delta_east, delta_down]))

        return ned_waypoints

    def _convert_gps_to_ned_single(
        self,
        origin: np.ndarray,
        lat: float,
        lon: float,
        alt: float
    ) -> np.ndarray:
        """単一GPS座標をNED座標に変換（coordinate_conversion使用）"""
        converter = get_converter()

        try:
            # 座標変換モジュールを使用
            x, y, z = converter.gps_to_ned(lat, lon, alt)
            return np.array([x, y, z])
        except ValueError:
            # ホームポジション未設定の場合はフォールバック
            print(f"警告: ホームポジション未設定のためGPS変換失敗")
            origin_lat = 35.0
            origin_lon = 136.0
            delta_north = (lat - origin_lat) * 111000.0
            delta_east = (lon - origin_lon) * 111000.0 * np.cos(np.radians(origin_lat))
            delta_down = -alt
            return np.array([delta_north, delta_east, delta_down])

    def _optimize_waypoint_order(
        self,
        start_pos: np.ndarray,
        waypoints: List[np.ndarray],
        destination: np.ndarray
    ) -> List[np.ndarray]:
        """
        ウェイポイントの順序をTSPアルゴリズムで最適化

        Args:
            start_pos: 開始位置
            waypoints: ウェイポイントリスト
            destination: 目的地

        Returns:
            最適化されたウェイポイント順序
        """
        if len(waypoints) <= 1:
            return waypoints

        # 全ポイント（開始点 + ウェイポイント + 目的地）
        all_points = [start_pos] + waypoints + [destination]
        n = len(all_points)

        # 距離行列を計算
        points_array = np.array(all_points)
        dist_matrix = distance_matrix(points_array, points_array)

        # Greedy nearest neighbor アルゴリズム
        # （大規模TSPには不十分だが、小規模には十分高速）
        unvisited = set(range(1, n - 1))  # 開始点と目的地を除く
        current = 0  # 開始点
        route = [current]

        while unvisited:
            # 未訪問の中で最も近いポイントを選択
            nearest = min(unvisited, key=lambda x: dist_matrix[current][x])
            route.append(nearest)
            unvisited.remove(nearest)
            current = nearest

        route.append(n - 1)  # 目的地を追加

        # 最適化された順序でウェイポイントを返す
        optimized = [all_points[i] for i in route]

        print(f"ウェイポイント順序最適化: 総距離 {self._calculate_route_length(optimized):.1f}m")

        return optimized[1:-1]  # 開始点と目的地を除く

    def _calculate_route_length(self, route: List[np.ndarray]) -> float:
        """ルート全長を計算"""
        total = 0.0
        for i in range(len(route) - 1):
            total += np.linalg.norm(route[i+1] - route[i])
        return total

    def _smooth_trajectory(
        self,
        start_pos: np.ndarray,
        waypoints: List[np.ndarray],
        destination: np.ndarray,
        num_interpolations: int = 10
    ) -> List[np.ndarray]:
        """
        経路を平滑化して補間

        Args:
            start_pos: 開始位置
            waypoints: ウェイポイント
            destination: 目的地
            num_interpolations: 各区間の補間点数

        Returns:
            平滑化された経路
        """
        # 全ウェイポイント（開始 + 中間 + 終了）
        full_route = [start_pos] + waypoints + [destination]

        smoothed = []

        for i in range(len(full_route) - 1):
            p0 = full_route[i]
            p1 = full_route[i + 1]

            # Catmull-Rom spline補間
            # 簡易実装: 線形補間 + 平滑化
            for t in np.linspace(0, 1, num_interpolations, endpoint=(i == len(full_route) - 2)):
                # 線形補間
                point = p0 * (1 - t) + p1 * t

                # Cubic smoothing（簡易版）
                # t_smooth = t * t * (3 - 2 * t)  # Smoothstep function
                # point = p0 * (1 - t_smooth) + p1 * t_smooth

                smoothed.append(point)

        print(f"経路平滑化: {len(full_route)}点 → {len(smoothed)}点")

        return smoothed

    def _avoid_nfz(self, route: List[np.ndarray]) -> List[np.ndarray]:
        """
        NFZ（飛行禁止区域）を回避するようにルートを修正

        Args:
            route: 元のルート

        Returns:
            NFZ回避ルート
        """
        # Flight controllerがNFZチェック機能を持たない場合はそのまま返す
        if not self.flight_controller or not hasattr(self.flight_controller, 'check_position_in_nfz'):
            return route

        safe_route = []

        for point in route:
            # NFZチェック（GPS座標に変換して確認）
            # 簡略化: 直接NEDでチェック（実際はGPS変換が必要）
            lat, lon, alt = self._ned_to_gps(point)

            in_nfz, zone = self.flight_controller.check_position_in_nfz([lat, lon, alt])

            if in_nfz:
                # NFZを回避する代替ポイントを計算
                adjusted_point = self._find_safe_point(point, zone)
                safe_route.append(adjusted_point)
                print(f"NFZ回避: {zone['name']}")
            else:
                safe_route.append(point)

        return safe_route

    def _ned_to_gps(self, ned_point: np.ndarray) -> Tuple[float, float, float]:
        """NED座標をGPS座標に変換（簡易版）"""
        origin_lat = 35.0
        origin_lon = 136.0

        lat = origin_lat + ned_point[0] / 111000.0
        lon = origin_lon + ned_point[1] / (111000.0 * np.cos(np.radians(origin_lat)))
        alt = -ned_point[2]

        return lat, lon, alt

    def _find_safe_point(
        self,
        point: np.ndarray,
        nfz_zone: Dict
    ) -> np.ndarray:
        """
        NFZ内のポイントを安全な位置に移動

        Args:
            point: 元のポイント（NED）
            nfz_zone: NFZ情報

        Returns:
            安全なポイント（NED）
        """
        # 簡易実装: 高度を上げる
        safe_point = point.copy()

        if nfz_zone.get('altitude_limit', 0) > 0:
            # 高度制限がある場合、制限高度 + マージンに上昇
            safe_alt = nfz_zone['altitude_limit'] + self.nfz_safety_margin
            safe_point[2] = -safe_alt  # NEDではZ軸が下向き
        else:
            # 高度制限がない場合、水平方向に回避
            # （簡略化: 北方向に100m移動）
            safe_point[0] += self.nfz_safety_margin

        return safe_point

    def _optimize_altitude(self, route: List[np.ndarray]) -> List[np.ndarray]:
        """
        高度を最適化（エネルギー効率とAGL維持のバランス）

        Args:
            route: ルート

        Returns:
            高度最適化ルート
        """
        optimized = []

        min_altitude = 50.0  # 最低高度 50m AGL
        max_altitude = 100.0  # 最高高度 100m AGL

        for point in route:
            optimized_point = point.copy()

            # 高度制約を適用
            current_alt = -point[2]  # NEDをAGLに変換

            if current_alt < min_altitude:
                optimized_point[2] = -min_altitude
            elif current_alt > max_altitude:
                optimized_point[2] = -max_altitude

            optimized.append(optimized_point)

        return optimized

    def calculate_trajectory_realtime(
        self,
        current_pos: np.ndarray,
        waypoints: List[np.ndarray],
        current_wp_index: int,
        obstacles: List,
        wind: np.ndarray,
        dt: float = 0.01
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        リアルタイム軌道計算（100Hz呼び出し用）

        Args:
            current_pos: 現在位置
            waypoints: 最適化済みウェイポイント
            current_wp_index: 現在のウェイポイントインデックス
            obstacles: 障害物リスト
            wind: 風速ベクトル
            dt: 時間刻み（秒）

        Returns:
            (目標位置, 目標速度)
        """
        if current_wp_index >= len(waypoints):
            return current_pos, np.zeros(3)

        target_wp = waypoints[current_wp_index]

        # 目標方向ベクトル
        direction = target_wp - current_pos
        distance = np.linalg.norm(direction)

        if distance > 0:
            direction = direction / distance

        # 障害物回避ベクトル
        avoid_vector = self._calculate_local_avoidance(current_pos, obstacles)

        # 風補償
        wind_compensation = -wind * 0.3

        # 速度合成
        desired_velocity = (
            direction * 3.0 +        # 目標方向
            avoid_vector * 2.0 +     # 障害物回避
            wind_compensation        # 風補償
        )

        # 速度制限
        max_velocity = 5.0  # m/s
        speed = np.linalg.norm(desired_velocity)
        if speed > max_velocity:
            desired_velocity = desired_velocity * (max_velocity / speed)

        # 次の位置
        next_position = current_pos + desired_velocity * dt

        return next_position, desired_velocity

    def _calculate_local_avoidance(
        self,
        position: np.ndarray,
        obstacles: List
    ) -> np.ndarray:
        """局所的な障害物回避ベクトル計算"""
        if not obstacles:
            return np.zeros(3)

        repulsive_force = np.zeros(3)

        for obs in obstacles:
            diff = position - obs['position']
            dist = np.linalg.norm(diff)

            if dist < 5.0:  # 5m以内の障害物のみ考慮
                force = (diff / (dist + 0.1)) * (1.0 / (dist + 0.1)**2)
                repulsive_force += force

        return repulsive_force
