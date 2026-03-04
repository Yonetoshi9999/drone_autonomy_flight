#!/usr/bin/env python3
"""
ルート最適化テストスクリプト
Mission PlannerからのMISSION_ITEMメッセージ受信とAI最適化をテスト
"""

import numpy as np
import sys
from route_optimizer import RouteOptimizer


def test_route_optimization():
    """ルート最適化の基本動作テスト"""
    print("=" * 60)
    print("ルート最適化テスト開始")
    print("=" * 60)

    # RouteOptimizerインスタンス作成（NFZチェックなし）
    optimizer = RouteOptimizer(flight_controller=None)

    # テストデータ
    start_pos = np.array([0.0, 0.0, -50.0])  # NED座標: 高度50m

    # 目的地（GPS座標）
    destination = {
        'lat': 35.001,  # 北に約111m
        'lon': 136.001,  # 東に約88m
        'alt': 60.0     # 高度60m
    }

    # ウェイポイント（GPS座標）
    waypoints = [
        {'lat': 35.0003, 'lon': 136.0003, 'alt': 55.0},
        {'lat': 35.0007, 'lon': 136.0007, 'alt': 58.0},
        {'lat': 35.0005, 'lon': 136.0008, 'alt': 62.0},
    ]

    print(f"\n開始位置: {start_pos}")
    print(f"目的地: ({destination['lat']:.6f}, {destination['lon']:.6f}, {destination['alt']}m)")
    print(f"ウェイポイント数: {len(waypoints)}")

    # ルート最適化実行
    try:
        optimized_route = optimizer.optimize_route(
            start_pos=start_pos,
            destination=destination,
            waypoints=waypoints,
            obstacles=None
        )

        print(f"\n最適化完了!")
        print(f"生成されたルートポイント数: {len(optimized_route)}")
        print(f"最初の5ポイント:")
        for i, point in enumerate(optimized_route[:5]):
            print(f"  {i+1}. ({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f})")

        # ルート全長計算
        total_distance = 0.0
        for i in range(len(optimized_route) - 1):
            segment = np.linalg.norm(optimized_route[i+1] - optimized_route[i])
            total_distance += segment

        print(f"\nルート全長: {total_distance:.2f}m")

        return True

    except Exception as e:
        print(f"\nエラー: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_realtime_trajectory():
    """リアルタイム軌道計算テスト（100Hz想定）"""
    print("\n" + "=" * 60)
    print("リアルタイム軌道計算テスト（100Hz）")
    print("=" * 60)

    optimizer = RouteOptimizer(flight_controller=None)

    # 簡単なウェイポイントリスト（NED座標）
    waypoints = [
        np.array([10.0, 0.0, -50.0]),
        np.array([20.0, 5.0, -55.0]),
        np.array([30.0, 10.0, -60.0]),
    ]

    current_pos = np.array([0.0, 0.0, -50.0])
    current_wp_index = 0

    # 障害物リスト（簡易）
    obstacles = [
        {'position': np.array([15.0, 2.0, -50.0]), 'radius': 3.0}
    ]

    # 風ベクトル（NED）
    wind = np.array([1.0, 0.5, 0.0])  # 北風1m/s、東風0.5m/s

    print(f"\n現在位置: {current_pos}")
    print(f"ウェイポイント数: {len(waypoints)}")
    print(f"障害物数: {len(obstacles)}")
    print(f"風速: {wind}")

    # 10回の軌道計算（100msシミュレーション）
    print(f"\n100Hzシミュレーション（10サイクル = 100ms）:")

    try:
        for i in range(10):
            next_pos, velocity = optimizer.calculate_trajectory_realtime(
                current_pos=current_pos,
                waypoints=waypoints,
                current_wp_index=current_wp_index,
                obstacles=obstacles,
                wind=wind,
                dt=0.01  # 10ms
            )

            # 位置更新
            current_pos = next_pos

            # ウェイポイント到達チェック
            if current_wp_index < len(waypoints):
                distance = np.linalg.norm(waypoints[current_wp_index] - current_pos)
                if distance < 2.0:
                    current_wp_index += 1
                    print(f"  サイクル {i+1}: ウェイポイント {current_wp_index} 到達!")

            if i % 3 == 0:  # 3サイクルごとに表示
                print(f"  サイクル {i+1}: 位置=({current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f}), "
                      f"速度={np.linalg.norm(velocity):.2f}m/s")

        print("\nリアルタイム計算成功!")
        return True

    except Exception as e:
        print(f"\nエラー: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_waypoint_order_optimization():
    """ウェイポイント順序最適化テスト（TSP）"""
    print("\n" + "=" * 60)
    print("ウェイポイント順序最適化テスト（TSP）")
    print("=" * 60)

    optimizer = RouteOptimizer(flight_controller=None)

    start_pos = np.array([0.0, 0.0, -50.0])
    destination = np.array([100.0, 100.0, -60.0])

    # ランダムな5個のウェイポイント
    waypoints = [
        np.array([80.0, 20.0, -55.0]),
        np.array([20.0, 80.0, -52.0]),
        np.array([50.0, 50.0, -58.0]),
        np.array([30.0, 30.0, -54.0]),
        np.array([70.0, 70.0, -56.0]),
    ]

    print(f"\n開始位置: {start_pos}")
    print(f"目的地: {destination}")
    print(f"ウェイポイント数: {len(waypoints)}")

    try:
        # 順序最適化
        optimized_order = optimizer._optimize_waypoint_order(
            start_pos, waypoints, destination
        )

        print(f"\n最適化された順序:")
        for i, wp in enumerate(optimized_order):
            print(f"  {i+1}. ({wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f})")

        # 距離計算
        route = [start_pos] + optimized_order + [destination]
        total_distance = 0.0
        for i in range(len(route) - 1):
            segment = np.linalg.norm(route[i+1] - route[i])
            total_distance += segment

        print(f"\n総移動距離: {total_distance:.2f}m")
        return True

    except Exception as e:
        print(f"\nエラー: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    print("RouteOptimizer テストスイート")
    print("=" * 60)

    results = []

    # テスト1: 基本的なルート最適化
    results.append(("ルート最適化", test_route_optimization()))

    # テスト2: リアルタイム軌道計算
    results.append(("リアルタイム軌道計算", test_realtime_trajectory()))

    # テスト3: ウェイポイント順序最適化
    results.append(("ウェイポイント順序最適化", test_waypoint_order_optimization()))

    # 結果サマリー
    print("\n" + "=" * 60)
    print("テスト結果サマリー")
    print("=" * 60)

    for name, result in results:
        status = "✓ 成功" if result else "✗ 失敗"
        print(f"{name}: {status}")

    # 全体結果
    all_passed = all(result for _, result in results)
    if all_passed:
        print("\n全テスト成功!")
        sys.exit(0)
    else:
        print("\n一部テスト失敗")
        sys.exit(1)
