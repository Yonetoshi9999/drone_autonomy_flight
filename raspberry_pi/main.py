#!/usr/bin/env python3
"""
空撮ドローン自律飛行システム - メインコントローラー (新仕様 v2.0)

新仕様:
- 50Hz障害物回避ループ (20ms周期)
- 20Hz制御コマンドループ (50ms周期)
- 10Hz状態管理ループ (100ms周期)
- 9状態ステートマシン統合
- TTC評価とスマート障害物回避
"""

import asyncio
import numpy as np
import time
import signal
import sys
from pathlib import Path
from pymavlink import mavutil

# 自作モジュール
from flight_controller import FlightController
from obstacle_avoidance import ObstacleAvoidance, CollisionRisk, AvoidanceDirection
from camera_control import CameraControl
from stabilization import GimbalStabilization
from sensor_drivers.rplidar import RPLidarDriver
from sensor_drivers.camera import CameraDriver
from autonomy_state import AutonomyStateManager, AutonomyState

class AerialPhotographyDrone:
    """メインドローンコントローラー (新仕様 v2.0)"""

    def __init__(self):
        print("空撮ドローンシステム起動中 (新仕様 v2.0)...")

        # MAVLink接続
        self.mavlink = mavutil.mavlink_connection(
            '/dev/ttyAMA0',
            baud=921600,
            source_system=1
        )

        # 各モジュール初期化
        self.flight = FlightController(self.mavlink)
        self.obstacle = ObstacleAvoidance()
        self.camera = CameraControl()
        self.gimbal = GimbalStabilization()

        # 自律飛行状態管理
        self.autonomy = AutonomyStateManager(self.mavlink, self.flight)

        # ルート最適化エンジンをFlightControllerに設定（循環参照回避）
        self.flight.set_route_optimizer(self.autonomy.route_optimizer)

        # センサードライバ
        self.lidar = RPLidarDriver('/dev/ttyUSB0')
        self.camera_driver = CameraDriver()

        # 制御周期設定（新仕様）
        self.obstacle_avoidance_period = 0.020  # 20ms (50Hz) - 障害物回避
        self.control_period = 0.050  # 50ms (20Hz) - 位置・速度コマンド送信
        self.state_period = 0.100    # 100ms (10Hz) - 状態管理用
        self.photo_period = 0.500    # 500ms (2Hz) - 撮影

        # シャットダウンハンドラ
        signal.signal(signal.SIGINT, self.shutdown)
        signal.signal(signal.SIGTERM, self.shutdown)

    async def obstacle_avoidance_loop(self):
        """障害物回避ループ (50Hz - 新仕様)"""

        while True:
            cycle_start = time.perf_counter()

            try:
                # 自律飛行アクティブ時のみ実行
                if self.autonomy.is_autonomous_active():
                    # センサーデータ取得
                    lidar_data = self.lidar.get_scan()
                    camera_data = self.camera_driver.get_frame()

                    # ドローン現在位置・速度取得
                    drone_position = np.array(self.autonomy.telemetry.local_position)
                    drone_velocity = np.array(self.autonomy.telemetry.local_velocity)
                    drone_altitude = -drone_position[2]  # NED: -Z = 高度

                    # 障害物検知・リスク評価
                    obstacles, max_risk = self.obstacle.detect_and_assess(
                        lidar_data,
                        camera_data,
                        drone_position,
                        drone_velocity
                    )

                    # リスクレベルに応じた処理
                    if max_risk == CollisionRisk.CRITICAL:
                        # CRITICAL: 即座に回避モードへ移行
                        if self.autonomy.get_current_state() == AutonomyState.FLYING:
                            # 回避方向選択
                            direction_type, direction_vector = self.obstacle.select_avoidance_direction(
                                obstacles,
                                drone_position,
                                drone_altitude,
                                lidar_data
                            )

                            # AVOIDING状態へ移行
                            self.autonomy.enter_avoiding_mode(direction_vector)
                            print(f"CRITICAL障害物検出 - {direction_type.name}方向へ回避")

                    elif max_risk == CollisionRisk.WARNING:
                        # WARNING: ログ記録のみ（予防的措置）
                        print(f"WARNING: {len([o for o in obstacles if o.risk_level == CollisionRisk.WARNING])}個の障害物を検出")

                    # 回避モード中の処理
                    if self.autonomy.get_current_state() == AutonomyState.AVOIDING:
                        # 回避完了チェック
                        if self.obstacle.check_avoidance_completion(
                            obstacles,
                            drone_position,
                            drone_velocity,
                            self.autonomy.waypoints,
                            self.autonomy.current_wp_index
                        ):
                            # 回避完了 - FLYING状態へ復帰
                            self.autonomy.exit_avoiding_mode()
                            print("障害物回避完了 - 通常飛行に復帰")

            except Exception as e:
                print(f"障害物回避エラー: {e}")

            # 50Hz周期維持（20ms）
            elapsed = time.perf_counter() - cycle_start
            if elapsed < self.obstacle_avoidance_period:
                await asyncio.sleep(self.obstacle_avoidance_period - elapsed)
            else:
                print(f"警告: 障害物回避周期オーバー {elapsed*1000:.1f}ms")

    async def main_control_loop(self):
        """メイン制御ループ（20Hz - 新仕様）"""

        while True:
            cycle_start = time.perf_counter()

            try:
                # 自律飛行アクティブ時のみ制御実行
                if self.autonomy.is_autonomous_active() or self.autonomy.gps_loss.is_active:
                    # FlightController状態を更新
                    self.flight.update_state_from_telemetry(self.autonomy.telemetry)

                    # ドローン現在状態取得
                    drone_position = np.array(self.autonomy.telemetry.local_position)
                    drone_velocity = np.array(self.autonomy.telemetry.local_velocity)

                    # 風速データ取得
                    wind_data = self.flight.get_wind_estimate()

                    # 状態に応じた目標位置・速度計算
                    if self.autonomy.get_current_state() == AutonomyState.AVOIDING:
                        # 回避モード: 回避方向への速度指令
                        avoidance_velocity = self.obstacle.get_avoidance_velocity(
                            AvoidanceDirection.LATERAL,  # デフォルト側方
                            self.autonomy.avoidance.avoidance_direction,
                            max_speed=3.0
                        )
                        target_pos = drone_position + avoidance_velocity * 0.05  # 50ms先
                        target_vel = avoidance_velocity

                    elif self.autonomy.gps_loss.is_active:
                        # GPS喪失時: ホバリング（状態管理側で送信）
                        target_pos = drone_position
                        target_vel = np.zeros(3)

                    else:
                        # 通常飛行: ウェイポイント追従
                        target_pos, target_vel = self.flight.calculate_trajectory(
                            [],  # 障害物は別ループで処理
                            wind_data
                        )

                    # FLYING/AVOIDING状態では高度40m以上を維持（新仕様）
                    current_state = self.autonomy.get_current_state()
                    if current_state in [AutonomyState.FLYING, AutonomyState.AVOIDING]:
                        # NED座標系: -z = 高度、40m以上を維持 → z <= -40.0
                        MIN_ALTITUDE = 40.0  # m
                        if target_pos[2] > -MIN_ALTITUDE:
                            target_pos[2] = -MIN_ALTITUDE
                            # 下方向への速度も制限
                            if target_vel[2] > 0:
                                target_vel[2] = 0.0
                            print(f"高度制約適用: {MIN_ALTITUDE}m以上を維持（状態: {current_state.name}）")

                    # 位置・速度・ヨー・ヨーレートコマンド送信（20Hz）
                    self.autonomy.send_position_velocity_command(
                        target_pos, target_vel
                    )

            except Exception as e:
                print(f"制御エラー: {e}")

            # 50ms周期維持（20Hz）
            elapsed = time.perf_counter() - cycle_start
            if elapsed < self.control_period:
                await asyncio.sleep(self.control_period - elapsed)
            else:
                print(f"警告: 制御周期オーバー {elapsed*1000:.1f}ms")

    async def photo_capture_loop(self):
        """撮影ループ（2Hz）"""

        while True:
            try:
                # 画像取得と保存
                self.camera.capture_and_save()
                await asyncio.sleep(self.photo_period)

            except Exception as e:
                print(f"撮影エラー: {e}")
                await asyncio.sleep(self.photo_period)

    async def state_management_loop(self):
        """状態管理ループ（10Hz - 新仕様）"""

        while True:
            cycle_start = time.perf_counter()

            try:
                # 状態更新（テレメトリ受信、状態遷移、ハートビート送信）
                self.autonomy.update_state()

                # 状態情報を定期的に表示（5秒ごと）
                if int(time.time()) % 5 == 0 and time.time() - cycle_start < 0.1:
                    state_info = self.autonomy.get_state_info()
                    print(f"[{state_info['state']}] "
                          f"WP={state_info['current_waypoint']}/{state_info['waypoint_count']}, "
                          f"GPS={state_info['telemetry']['gps_satellites']}衛星, "
                          f"BAT={state_info['telemetry']['battery_remaining']}%, "
                          f"ALT={state_info['telemetry']['relative_alt']:.1f}m, "
                          f"避:{state_info['avoidance_active']}")

            except Exception as e:
                print(f"状態管理エラー: {e}")

            # 10Hz周期維持（100ms）
            elapsed = time.perf_counter() - cycle_start
            if elapsed < self.state_period:
                await asyncio.sleep(self.state_period - elapsed)
            else:
                print(f"警告: 状態管理周期オーバー {elapsed*1000:.1f}ms")

    async def run(self):
        """システム実行"""

        # 初期化
        await self.initialize_sensors()

        print("\n" + "="*70)
        print(" 空撮ドローン自律飛行システム v2.0 起動")
        print("="*70)
        print("制御周期:")
        print(f"  - 障害物回避: 50Hz (20ms)")
        print(f"  - 制御コマンド: 20Hz (50ms)")
        print(f"  - 状態管理: 10Hz (100ms)")
        print(f"  - 撮影: 2Hz (500ms)")
        print("="*70 + "\n")

        # 並行実行 (4つの独立したループ)
        tasks = [
            asyncio.create_task(self.obstacle_avoidance_loop()),  # 50Hz
            asyncio.create_task(self.main_control_loop()),         # 20Hz
            asyncio.create_task(self.state_management_loop()),     # 10Hz
            asyncio.create_task(self.photo_capture_loop()),        # 2Hz
        ]

        await asyncio.gather(*tasks)

    async def initialize_sensors(self):
        """センサー初期化"""
        self.lidar.connect()
        self.camera_driver.initialize()
        print("センサー初期化完了")

    def shutdown(self, signum, frame):
        """シャットダウン処理"""
        print("\nシステム終了中...")
        self.lidar.stop()
        self.camera_driver.release()
        sys.exit(0)

if __name__ == "__main__":
    drone = AerialPhotographyDrone()
    asyncio.run(drone.run())
