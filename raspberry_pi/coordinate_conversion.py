"""
座標変換モジュール
GPS座標 ⇔ Local NED座標の相互変換

仕様:
- ホームポジション基準のNED座標系
- 簡易球面近似計算（精度: ±1m以内、数km範囲）
- 高精度が必要な場合はpyprojライブラリを使用可能
"""

import numpy as np
import math
from typing import Tuple


class CoordinateConverter:
    """座標変換クラス"""

    # 地球半径 (m)
    EARTH_RADIUS = 6378137.0  # WGS84楕円体の赤道半径

    # 1度あたりの距離 (赤道付近)
    METERS_PER_DEGREE_LAT = 111320.0  # 緯度1度 ≈ 111.32km

    def __init__(self):
        """初期化"""
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None
        self.is_home_set = False

    def set_home_position(self, lat: float, lon: float, alt: float):
        """
        ホームポジション設定

        Args:
            lat: 緯度 (度)
            lon: 経度 (度)
            alt: 高度 (m, 海抜)
        """
        self.home_lat = lat
        self.home_lon = lon
        self.home_alt = alt
        self.is_home_set = True
        print(f"ホームポジション設定: ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")

    def gps_to_ned(self, lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
        """
        GPS座標 → Local NED座標変換

        Args:
            lat: 緯度 (度)
            lon: 経度 (度)
            alt: 高度 (m, 海抜)

        Returns:
            (x, y, z): NED座標 (m)
                x: 北方向 (North)
                y: 東方向 (East)
                z: 下方向 (Down, 地上は負値)

        Raises:
            ValueError: ホームポジション未設定時
        """
        if not self.is_home_set:
            raise ValueError("ホームポジションが設定されていません")

        # 緯度差 → 北方向距離 (x)
        # 1度 ≈ 111.32km (緯度による変化は小さいので一定値使用)
        x = (lat - self.home_lat) * self.METERS_PER_DEGREE_LAT

        # 経度差 → 東方向距離 (y)
        # 緯度により経度1度あたりの距離が変化: cos(緯度)で補正
        meters_per_degree_lon = self.METERS_PER_DEGREE_LAT * math.cos(math.radians(self.home_lat))
        y = (lon - self.home_lon) * meters_per_degree_lon

        # 高度差 → 下方向距離 (z)
        # 下向きが正なので符号反転
        z = -(alt - self.home_alt)

        return (x, y, z)

    def ned_to_gps(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """
        Local NED座標 → GPS座標変換

        Args:
            x: 北方向距離 (m)
            y: 東方向距離 (m)
            z: 下方向距離 (m, 地上は負値)

        Returns:
            (lat, lon, alt): GPS座標
                lat: 緯度 (度)
                lon: 経度 (度)
                alt: 高度 (m, 海抜)

        Raises:
            ValueError: ホームポジション未設定時
        """
        if not self.is_home_set:
            raise ValueError("ホームポジションが設定されていません")

        # 北方向距離 → 緯度差
        lat = self.home_lat + (x / self.METERS_PER_DEGREE_LAT)

        # 東方向距離 → 経度差
        meters_per_degree_lon = self.METERS_PER_DEGREE_LAT * math.cos(math.radians(self.home_lat))
        lon = self.home_lon + (y / meters_per_degree_lon)

        # 下方向距離 → 高度差 (符号反転)
        alt = self.home_alt - z

        return (lat, lon, alt)

    def distance_3d(self, lat1: float, lon1: float, alt1: float,
                    lat2: float, lon2: float, alt2: float) -> float:
        """
        2点間の3D距離計算 (GPS座標)

        Args:
            lat1, lon1, alt1: 点1のGPS座標
            lat2, lon2, alt2: 点2のGPS座標

        Returns:
            float: 3D距離 (m)
        """
        # 一時的にホームポジションを設定
        original_home = (self.home_lat, self.home_lon, self.home_alt, self.is_home_set)

        self.set_home_position(lat1, lon1, alt1)
        x, y, z = self.gps_to_ned(lat2, lon2, alt2)
        distance = math.sqrt(x**2 + y**2 + z**2)

        # ホームポジションを復元
        if original_home[3]:
            self.home_lat, self.home_lon, self.home_alt, self.is_home_set = original_home
        else:
            self.is_home_set = False

        return distance

    def distance_2d(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        2点間の水平距離計算 (GPS座標)

        Args:
            lat1, lon1: 点1の緯度・経度
            lat2, lon2: 点2の緯度・経度

        Returns:
            float: 水平距離 (m)
        """
        # 一時的にホームポジションを設定
        original_home = (self.home_lat, self.home_lon, self.home_alt, self.is_home_set)

        self.set_home_position(lat1, lon1, 0)
        x, y, _ = self.gps_to_ned(lat2, lon2, 0)
        distance = math.sqrt(x**2 + y**2)

        # ホームポジションを復元
        if original_home[3]:
            self.home_lat, self.home_lon, self.home_alt, self.is_home_set = original_home
        else:
            self.is_home_set = False

        return distance

    def get_home_position(self) -> Tuple[float, float, float]:
        """
        ホームポジション取得

        Returns:
            (lat, lon, alt): ホームポジションGPS座標

        Raises:
            ValueError: ホームポジション未設定時
        """
        if not self.is_home_set:
            raise ValueError("ホームポジションが設定されていません")

        return (self.home_lat, self.home_lon, self.home_alt)

    def reset_home_position(self):
        """ホームポジションリセット"""
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None
        self.is_home_set = False
        print("ホームポジションをリセットしました")


# グローバルインスタンス（シングルトンパターン）
_converter_instance = None

def get_converter() -> CoordinateConverter:
    """
    座標変換インスタンス取得（シングルトン）

    Returns:
        CoordinateConverter: 座標変換インスタンス
    """
    global _converter_instance
    if _converter_instance is None:
        _converter_instance = CoordinateConverter()
    return _converter_instance


# 便利な関数エイリアス
def set_home(lat: float, lon: float, alt: float):
    """ホームポジション設定（便利関数）"""
    get_converter().set_home_position(lat, lon, alt)

def gps_to_ned(lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
    """GPS → NED変換（便利関数）"""
    return get_converter().gps_to_ned(lat, lon, alt)

def ned_to_gps(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """NED → GPS変換（便利関数）"""
    return get_converter().ned_to_gps(x, y, z)

def get_home() -> Tuple[float, float, float]:
    """ホームポジション取得（便利関数）"""
    return get_converter().get_home_position()


if __name__ == "__main__":
    """テストコード"""
    print("=== 座標変換モジュール テスト ===\n")

    # テスト用座標（名古屋付近）
    home_lat = 35.1814
    home_lon = 136.9063
    home_alt = 50.0

    # ホームポジション設定
    converter = CoordinateConverter()
    converter.set_home_position(home_lat, home_lon, home_alt)

    # GPS → NED変換テスト
    print("1. GPS → NED変換テスト")
    test_lat = home_lat + 0.001  # 約111m北
    test_lon = home_lon + 0.001  # 約88m東（名古屋の緯度）
    test_alt = 100.0  # 50m上空

    x, y, z = converter.gps_to_ned(test_lat, test_lon, test_alt)
    print(f"  GPS: ({test_lat:.6f}, {test_lon:.6f}, {test_alt:.1f}m)")
    print(f"  NED: ({x:.2f}m, {y:.2f}m, {z:.2f}m)")
    print(f"  期待値: (約111m, 約88m, 約-50m)\n")

    # NED → GPS逆変換テスト
    print("2. NED → GPS逆変換テスト")
    lat_back, lon_back, alt_back = converter.ned_to_gps(x, y, z)
    print(f"  元のGPS: ({test_lat:.6f}, {test_lon:.6f}, {test_alt:.1f}m)")
    print(f"  逆変換:  ({lat_back:.6f}, {lon_back:.6f}, {alt_back:.1f}m)")
    print(f"  誤差: ({abs(test_lat-lat_back)*1e6:.2f}μ度, "
          f"{abs(test_lon-lon_back)*1e6:.2f}μ度, "
          f"{abs(test_alt-alt_back):.3f}m)\n")

    # 距離計算テスト
    print("3. 距離計算テスト")
    distance_3d = converter.distance_3d(home_lat, home_lon, home_alt,
                                         test_lat, test_lon, test_alt)
    distance_2d = converter.distance_2d(home_lat, home_lon, test_lat, test_lon)
    print(f"  3D距離: {distance_3d:.2f}m")
    print(f"  2D距離: {distance_2d:.2f}m")
    print(f"  期待値: 3D≈143m, 2D≈141m\n")

    # 便利関数テスト
    print("4. 便利関数テスト")
    set_home(home_lat, home_lon, home_alt)
    x2, y2, z2 = gps_to_ned(test_lat, test_lon, test_alt)
    print(f"  便利関数経由: ({x2:.2f}m, {y2:.2f}m, {z2:.2f}m)")
    print(f"  直接呼び出し: ({x:.2f}m, {y:.2f}m, {z:.2f}m)")
    print(f"  一致: {x==x2 and y==y2 and z==z2}\n")

    print("=== テスト完了 ===")
