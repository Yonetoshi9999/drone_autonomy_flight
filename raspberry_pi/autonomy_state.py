"""
自律飛行状態管理モジュール (新仕様 v2.0)
Mode 99での自律飛行における状態遷移とMAVLink通信を管理

新仕様:
- 9状態システム: INIT(0), READY(1), FLYING(2), AVOIDING(3), REPLANNING(4), HOVERING(5), LANDING(6), ERROR(99)
- 20Hz制御コマンド送信 (位置・速度・ヨー・ヨーレート)
- 50Hz障害物回避との統合
- 包括的なプリフライトチェック
- GPS喪失ホバリングモード
"""

import time
import numpy as np
import logging
from enum import Enum
from pymavlink import mavutil
from dataclasses import dataclass
from typing import Optional, List, Tuple, Dict
from route_optimizer import RouteOptimizer
from coordinate_conversion import CoordinateConverter

# ロギング設定
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('/home/pi/aerial_photography_drone/logs/autonomy_state.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


class AutonomyState(Enum):
    """自律飛行状態 (新仕様 9状態)"""
    INIT = 0          # 初期化 (NFZ取得 + 経路計画)
    READY = 1         # 準備完了 (プリフライトチェック完了)
    FLYING = 2        # 自律飛行中
    AVOIDING = 3      # 障害物回避中
    REPLANNING = 4    # 経路再計算中
    HOVERING = 5      # GPS喪失ホバリング
    LANDING = 6       # 着陸シーケンス
    ERROR = 99        # エラー状態 (統合エラー)


@dataclass
class TelemetryData:
    """テレメトリデータ（MAVLinkタイムスタンプ付き）"""
    # GPS
    gps_satellites: int = 0
    gps_hdop: float = 99.9
    gps_fix_type: int = 0

    # バッテリー
    battery_remaining: int = 0
    battery_voltage: float = 0.0

    # EKF
    ekf_flags: int = 0
    ekf_velocity_variance: float = 0.0
    ekf_pos_horiz_variance: float = 0.0

    # 位置・姿勢
    global_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # lat, lon, alt
    local_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)   # NED
    local_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)   # NED
    attitude: Tuple[float, float, float] = (0.0, 0.0, 0.0)         # roll, pitch, yaw
    relative_alt: float = 0.0  # m

    # センサーヘルス
    sensors_health: int = 0

    # MAVLinkタイムスタンプ (ms)
    timestamp_position: int = 0  # LOCAL_POSITION_NED
    timestamp_attitude: int = 0  # ATTITUDE
    timestamp_gps: int = 0       # GPS_RAW_INT

    # 最終更新時刻 (システム時刻)
    last_heartbeat: float = 0.0
    last_update: float = 0.0


@dataclass
class GPSLossData:
    """GPS喪失時のデータ"""
    loss_time: float = 0.0
    last_valid_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # NED
    last_valid_altitude: float = 0.0  # m
    timeout_duration: float = 30.0    # 秒
    is_active: bool = False


@dataclass
class AvoidanceData:
    """障害物回避データ"""
    is_active: bool = False
    avoidance_start_time: float = 0.0
    original_waypoint_index: int = 0
    avoidance_direction: np.ndarray = None

    def __post_init__(self):
        if self.avoidance_direction is None:
            self.avoidance_direction = np.zeros(3)


class AutonomyStateManager:
    """自律飛行状態管理クラス (新仕様 v2.0)"""

    def __init__(self, mavlink_connection, flight_controller):
        """
        初期化

        Args:
            mavlink_connection: MAVLink接続
            flight_controller: FlightControllerインスタンス
        """
        self.mavlink = mavlink_connection
        self.flight = flight_controller

        # 状態管理
        self.state = AutonomyState.INIT
        self.previous_state = None

        # テレメトリデータ
        self.telemetry = TelemetryData()

        # GPS喪失管理
        self.gps_loss = GPSLossData()

        # 障害物回避管理
        self.avoidance = AvoidanceData()

        # 通信タイミング
        self.control_period = 0.05       # 50ms = 20Hz (制御コマンド)
        self.heartbeat_period = 1.0      # 1Hz (ハートビート)
        self.state_update_period = 0.1   # 10Hz (状態更新)

        self.last_control_send = time.time()
        self.last_heartbeat_send = time.time()
        self.last_state_update = time.time()

        self.heartbeat_timeout = 5.0     # 5秒でタイムアウト (新仕様)

        # ミッション管理
        self.mission_items = []          # Mission Plannerから受信したウェイポイント
        self.mission_count = 0
        self.mission_ready = False       # USER_MISSION_RDY パラメータ
        self.receiving_mission = False

        self.waypoints = []              # AI最適化後のウェイポイント
        self.current_wp_index = 0

        # AIルート最適化
        self.route_optimizer = RouteOptimizer(flight_controller)

        # 座標変換
        self.coord_converter = CoordinateConverter()
        self.home_position_set = False

        # アーム・離陸パラメータ
        self.takeoff_altitude = 50.0     # m
        self.altitude_tolerance = 5.0    # m (45-50mで到達判定)

        # プリフライトチェック結果
        self.preflight_results: Dict[str, bool] = {}
        self.preflight_errors: List[str] = []

        # 初期化完了フラグ
        self.init_complete = False
        self.nfz_loaded = False
        self.path_planned = False

        logger.info("自律飛行状態管理システム初期化完了 (新仕様 v2.0)")

    # ============================================================================
    # MAVLink通信: 受信
    # ============================================================================

    def receive_telemetry(self):
        """テレメトリメッセージを受信して更新（タイムスタンプ付き）"""

        # HEARTBEAT
        msg = self.mavlink.recv_match(type='HEARTBEAT', blocking=False)
        if msg:
            self.telemetry.last_heartbeat = time.time()

        # GPS_RAW_INT
        msg = self.mavlink.recv_match(type='GPS_RAW_INT', blocking=False)
        if msg:
            self.telemetry.gps_satellites = msg.satellites_visible
            self.telemetry.gps_hdop = msg.eph / 100.0  # cm → m
            self.telemetry.gps_fix_type = msg.fix_type
            self.telemetry.timestamp_gps = msg.time_usec // 1000  # us → ms

        # SYS_STATUS
        msg = self.mavlink.recv_match(type='SYS_STATUS', blocking=False)
        if msg:
            self.telemetry.battery_remaining = msg.battery_remaining
            self.telemetry.battery_voltage = msg.voltage_battery / 1000.0  # mV → V
            self.telemetry.sensors_health = msg.onboard_control_sensors_health

        # GLOBAL_POSITION_INT
        msg = self.mavlink.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000.0  # mm → m
            self.telemetry.global_position = (lat, lon, alt)
            self.telemetry.relative_alt = msg.relative_alt / 1000.0  # mm → m

            # ホームポジション設定 (初回GPS Fix時)
            if not self.home_position_set and self.telemetry.gps_fix_type >= 3:
                self.coord_converter.set_home_position(lat, lon, alt)
                self.home_position_set = True
                logger.info(f"ホームポジション設定完了: ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")

        # LOCAL_POSITION_NED
        msg = self.mavlink.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg:
            self.telemetry.local_position = (msg.x, msg.y, msg.z)
            self.telemetry.local_velocity = (msg.vx, msg.vy, msg.vz)
            self.telemetry.timestamp_position = msg.time_boot_ms  # ms

        # ATTITUDE
        msg = self.mavlink.recv_match(type='ATTITUDE', blocking=False)
        if msg:
            self.telemetry.attitude = (msg.roll, msg.pitch, msg.yaw)
            self.telemetry.timestamp_attitude = msg.time_boot_ms  # ms

        # EKF_STATUS_REPORT
        msg = self.mavlink.recv_match(type='EKF_STATUS_REPORT', blocking=False)
        if msg:
            self.telemetry.ekf_flags = msg.flags
            self.telemetry.ekf_velocity_variance = msg.velocity_variance
            self.telemetry.ekf_pos_horiz_variance = msg.pos_horiz_variance

        self.telemetry.last_update = time.time()

    def receive_mission_items(self):
        """Mission PlannerからMISSION_ITEMメッセージを受信"""

        # MISSION_COUNT メッセージを受信
        msg = self.mavlink.recv_match(type='MISSION_COUNT', blocking=False)
        if msg and hasattr(msg, 'count'):
            self.mission_count = msg.count
            self.mission_items = []
            self.receiving_mission = True
            logger.info(f"ミッション受信開始: {self.mission_count}個のウェイポイント")

            # MISSION_REQUEST_INTを送信して各アイテムを要求
            for i in range(self.mission_count):
                self.mavlink.mav.mission_request_int_send(
                    self.mavlink.target_system,
                    self.mavlink.target_component,
                    i,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                )
            return

        # MISSION_ITEM_INT メッセージを受信
        msg = self.mavlink.recv_match(type='MISSION_ITEM_INT', blocking=False)

        if msg and self.receiving_mission:
            # ウェイポイント情報を抽出
            lat = msg.x / 1e7
            lon = msg.y / 1e7
            alt = msg.z  # 高度（メートル）

            waypoint = {
                'seq': msg.seq,
                'lat': lat,
                'lon': lon,
                'alt': alt,
                'command': msg.command
            }

            self.mission_items.append(waypoint)
            logger.info(f"ウェイポイント {msg.seq + 1}/{self.mission_count} 受信: "
                       f"({lat:.6f}, {lon:.6f}, {alt:.1f}m)")

            # 最後のウェイポイントを受信したらMISSION_ACKを送信
            if len(self.mission_items) == self.mission_count:
                self.receiving_mission = False

                # MISSION_ACKを送信
                self.mavlink.mav.mission_ack_send(
                    self.mavlink.target_system,
                    self.mavlink.target_component,
                    mavutil.mavlink.MAV_MISSION_RESULT_ACCEPTED,
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION
                )

                logger.info(f"ミッション受信完了: {len(self.mission_items)}個のウェイポイント")

    def check_mission_ready_parameter(self):
        """USER_MISSION_RDYパラメータをチェック"""

        # パラメータ読み取りリクエスト送信
        self.mavlink.mav.param_request_read_send(
            self.mavlink.target_system,
            self.mavlink.target_component,
            b'USER_MISSION_RDY',
            -1
        )

        # PARAM_VALUE 受信
        msg = self.mavlink.recv_match(type='PARAM_VALUE', blocking=False, timeout=0.1)
        if msg and msg.param_id == 'USER_MISSION_RDY':
            self.mission_ready = (msg.param_value == 1)
            if self.mission_ready:
                logger.info("USER_MISSION_RDY = 1 検出: ミッション設定完了")

    # ============================================================================
    # MAVLink通信: 送信
    # ============================================================================

    def send_heartbeat(self):
        """ハートビート送信 (1Hz) - 状態情報付き"""
        current_time = time.time()

        if current_time - self.last_heartbeat_send >= self.heartbeat_period:
            # カスタムモードに状態を含める
            custom_mode = self.state.value

            # システムステータス決定
            if self.state == AutonomyState.ERROR:
                system_status = mavutil.mavlink.MAV_STATE_CRITICAL
            elif self.state == AutonomyState.AVOIDING:
                system_status = mavutil.mavlink.MAV_STATE_EMERGENCY
            else:
                system_status = mavutil.mavlink.MAV_STATE_ACTIVE

            self.mavlink.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, custom_mode, system_status
            )
            self.last_heartbeat_send = current_time

    def send_position_velocity_command(self, position: np.ndarray, velocity: np.ndarray):
        """位置・速度・ヨー・ヨーレートコマンド送信 (20Hz)"""
        current_time = time.time()

        if current_time - self.last_control_send >= self.control_period:
            # ヨー角を速度ベクトルから計算
            yaw = self._calculate_yaw_from_velocity(velocity)

            # ヨーレートをS字カーブで計算 (最大45°/s = 0.785 rad/s)
            yaw_rate = self._calculate_yaw_rate_sigmoid(velocity)

            # type_mask: 位置・速度・ヨー・ヨーレートを使用
            # 0b0000111111000111 = 位置(XYZ), 速度(XYZ), ヨー, ヨーレート有効
            type_mask = 0b0000111111000111

            self.mavlink.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self.mavlink.target_system,
                self.mavlink.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                position[0], position[1], position[2],
                velocity[0], velocity[1], velocity[2],
                0, 0, 0,  # accel (未使用)
                yaw, yaw_rate
            )
            self.last_control_send = current_time

    def _calculate_yaw_from_velocity(self, velocity: np.ndarray) -> float:
        """速度ベクトルからヨー角を計算"""
        vx, vy, vz = velocity
        speed = np.sqrt(vx**2 + vy**2)

        if speed < 0.1:  # 速度が小さい場合は現在のヨーを維持
            return self.telemetry.attitude[2]

        # atan2でヨー角計算 (北を0度として時計回り)
        yaw = np.arctan2(vy, vx)
        return yaw

    def _calculate_yaw_rate_sigmoid(self, velocity: np.ndarray) -> float:
        """S字カーブ (シグモイド) でヨーレートを計算"""
        vx, vy, vz = velocity
        speed = np.sqrt(vx**2 + vy**2)

        # 目標ヨー
        target_yaw = np.arctan2(vy, vx)
        current_yaw = self.telemetry.attitude[2]

        # ヨー差分計算 (-π ~ π に正規化)
        yaw_error = target_yaw - current_yaw
        yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))

        # シグモイド関数で滑らかなヨーレート生成
        # yaw_rate = max_rate * tanh(k * error)
        max_yaw_rate = 0.785  # 45°/s
        k = 2.0  # ゲイン

        yaw_rate = max_yaw_rate * np.tanh(k * yaw_error)

        return yaw_rate

    def send_arm_command(self):
        """アームコマンド送信"""
        logger.info("アームコマンド送信")
        self.mavlink.mav.command_long_send(
            self.mavlink.target_system,
            self.mavlink.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # param1: 1=arm
            0, 0, 0, 0, 0, 0
        )

    def send_disarm_command(self):
        """ディスアームコマンド送信"""
        logger.info("ディスアームコマンド送信")
        self.mavlink.mav.command_long_send(
            self.mavlink.target_system,
            self.mavlink.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            0,  # param1: 0=disarm
            0, 0, 0, 0, 0, 0
        )

    def send_takeoff_command(self, altitude: float):
        """離陸コマンド送信"""
        logger.info(f"離陸コマンド送信: 目標高度 {altitude}m")
        self.mavlink.mav.command_long_send(
            self.mavlink.target_system,
            self.mavlink.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # confirmation
            0, 0, 0, 0, 0, 0,
            altitude  # param7: 目標高度
        )

    def send_land_command(self):
        """着陸コマンド送信"""
        logger.info("着陸コマンド送信")
        self.mavlink.mav.command_long_send(
            self.mavlink.target_system,
            self.mavlink.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,  # confirmation
            0, 0, 0, 0, 0, 0, 0
        )

    def send_set_mode(self, mode_name: str):
        """モード変更コマンド送信"""
        logger.info(f"モード変更: {mode_name}")

        # ArduCopterのモードマッピング
        mode_mapping = {
            'STABILIZE': 0,
            'LAND': 9,
            'SMART_PHOTO': 99  # Mode 99
        }

        if mode_name in mode_mapping:
            custom_mode = mode_mapping[mode_name]
            self.mavlink.mav.set_mode_send(
                self.mavlink.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode
            )

    # ============================================================================
    # プリフライトチェック
    # ============================================================================

    def run_preflight_checks(self) -> bool:
        """
        包括的なプリフライトチェック実行

        Returns:
            bool: 全チェック項目合格時True
        """
        logger.info("=" * 70)
        logger.info("プリフライトチェック開始")
        logger.info("=" * 70)

        self.preflight_results = {}
        self.preflight_errors = []
        all_passed = True

        # GPS チェック
        gps_passed = self._check_gps()
        all_passed = all_passed and gps_passed

        # バッテリーチェック
        battery_passed = self._check_battery()
        all_passed = all_passed and battery_passed

        # EKF チェック
        ekf_passed = self._check_ekf()
        all_passed = all_passed and ekf_passed

        # センサーチェック
        sensor_passed = self._check_sensors()
        all_passed = all_passed and sensor_passed

        # 経路妥当性チェック
        path_passed = self._check_path_validity()
        all_passed = all_passed and path_passed

        # 結果サマリー
        logger.info("-" * 70)
        if all_passed:
            logger.info("✓ 全プリフライトチェック合格")
        else:
            logger.error("✗ プリフライトチェック失敗:")
            for error in self.preflight_errors:
                logger.error(f"  - {error}")
        logger.info("=" * 70)

        return all_passed

    def _check_gps(self) -> bool:
        """GPSチェック"""
        passed = True

        # GPS衛星数
        if self.telemetry.gps_satellites >= 10:
            logger.info(f"✓ GPS衛星数: {self.telemetry.gps_satellites} (必要: 10以上)")
            self.preflight_results['gps_satellites'] = True
        else:
            error_msg = f"GPS衛星数不足 (現在{self.telemetry.gps_satellites}、必要10以上)"
            logger.error(f"✗ {error_msg}")
            self.preflight_errors.append(error_msg)
            self.preflight_results['gps_satellites'] = False
            passed = False

        # GPS HDOP
        if self.telemetry.gps_hdop <= 1.5:
            logger.info(f"✓ GPS HDOP: {self.telemetry.gps_hdop:.2f} (必要: 1.5以下)")
            self.preflight_results['gps_hdop'] = True
        else:
            error_msg = f"GPS精度不足 (HDOP: {self.telemetry.gps_hdop:.2f}、必要1.5以下)"
            logger.error(f"✗ {error_msg}")
            self.preflight_errors.append(error_msg)
            self.preflight_results['gps_hdop'] = False
            passed = False

        # GPS Fix Type
        if self.telemetry.gps_fix_type >= 3:
            logger.info(f"✓ GPS Fix Type: {self.telemetry.gps_fix_type} (必要: 3以上)")
            self.preflight_results['gps_fix'] = True
        else:
            error_msg = f"GPS Fix未確立 (現在タイプ{self.telemetry.gps_fix_type}、必要3以上)"
            logger.error(f"✗ {error_msg}")
            self.preflight_errors.append(error_msg)
            self.preflight_results['gps_fix'] = False
            passed = False

        return passed

    def _check_battery(self) -> bool:
        """バッテリーチェック"""
        passed = True

        # バッテリー残量
        if self.telemetry.battery_remaining >= 90:
            logger.info(f"✓ バッテリー残量: {self.telemetry.battery_remaining}% (必要: 90%以上)")
            self.preflight_results['battery_remaining'] = True
        else:
            error_msg = f"バッテリー残量不足 (現在{self.telemetry.battery_remaining}%、必要90%以上)"
            logger.error(f"✗ {error_msg}")
            self.preflight_errors.append(error_msg)
            self.preflight_results['battery_remaining'] = False
            passed = False

        # バッテリー電圧 (4Sセルと仮定: 3.8V × 4 = 15.2V)
        cell_count = 4
        required_voltage = 3.8 * cell_count
        if self.telemetry.battery_voltage >= required_voltage:
            logger.info(f"✓ バッテリー電圧: {self.telemetry.battery_voltage:.2f}V (必要: {required_voltage:.2f}V以上)")
            self.preflight_results['battery_voltage'] = True
        else:
            error_msg = f"バッテリー電圧不足 (現在{self.telemetry.battery_voltage:.2f}V、必要{required_voltage:.2f}V以上)"
            logger.error(f"✗ {error_msg}")
            self.preflight_errors.append(error_msg)
            self.preflight_results['battery_voltage'] = False
            passed = False

        return passed

    def _check_ekf(self) -> bool:
        """EKFチェック"""
        passed = True

        # EKF フラグチェック
        required_flags = 0b00111111  # bits 0-5 (attitude, vel_horiz, vel_vert, pos_horiz_rel, pos_horiz_abs, pos_vert)
        flags_ok = (self.telemetry.ekf_flags & required_flags) == required_flags

        if flags_ok:
            logger.info(f"✓ EKFフラグ: 0x{self.telemetry.ekf_flags:04x} (全項目OK)")
            self.preflight_results['ekf_flags'] = True
        else:
            error_msg = "EKF状態異常 (一部推定値が不安定)"
            logger.error(f"✗ {error_msg}")
            self.preflight_errors.append(error_msg)
            self.preflight_results['ekf_flags'] = False
            passed = False

        # EKF Innovation チェック
        gate_size = 5.0
        vel_ratio = np.sqrt(self.telemetry.ekf_velocity_variance) / gate_size
        pos_ratio = np.sqrt(self.telemetry.ekf_pos_horiz_variance) / gate_size

        if vel_ratio < 0.5 and pos_ratio < 0.5:
            logger.info(f"✓ EKF Innovation: vel_ratio={vel_ratio:.2f}, pos_ratio={pos_ratio:.2f} (必要: <0.5)")
            self.preflight_results['ekf_innovation'] = True
        else:
            error_msg = f"EKF Innovation値異常 (vel_ratio: {vel_ratio:.2f}, pos_ratio: {pos_ratio:.2f})"
            logger.error(f"✗ {error_msg}")
            self.preflight_errors.append(error_msg)
            self.preflight_results['ekf_innovation'] = False
            passed = False

        return passed

    def _check_sensors(self) -> bool:
        """センサーヘルスチェック"""
        passed = True

        # IMU (加速度計・ジャイロ)
        imu_ok = (self.telemetry.sensors_health &
                  (mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO |
                   mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL)) != 0

        if imu_ok:
            logger.info("✓ IMUセンサー: 正常")
            self.preflight_results['imu'] = True
        else:
            error_msg = "IMUセンサー異常 (キャリブレーション必要)"
            logger.error(f"✗ {error_msg}")
            self.preflight_errors.append(error_msg)
            self.preflight_results['imu'] = False
            passed = False

        # 気圧計
        baro_ok = (self.telemetry.sensors_health &
                   mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE) != 0

        if baro_ok:
            logger.info("✓ 気圧計: 正常")
            self.preflight_results['barometer'] = True
        else:
            error_msg = "気圧計異常"
            logger.error(f"✗ {error_msg}")
            self.preflight_errors.append(error_msg)
            self.preflight_results['barometer'] = False
            passed = False

        # コンパス
        mag_ok = (self.telemetry.sensors_health &
                  mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG) != 0

        if mag_ok:
            logger.info("✓ コンパス: 正常")
            self.preflight_results['compass'] = True
        else:
            error_msg = "コンパス異常 (キャリブレーション必要)"
            logger.error(f"✗ {error_msg}")
            self.preflight_errors.append(error_msg)
            self.preflight_results['compass'] = False
            passed = False

        return passed

    def _check_path_validity(self) -> bool:
        """経路妥当性チェック"""
        passed = True

        if not self.path_planned or len(self.waypoints) == 0:
            error_msg = "経路が計画されていません"
            logger.error(f"✗ {error_msg}")
            self.preflight_errors.append(error_msg)
            self.preflight_results['path_validity'] = False
            return False

        logger.info(f"✓ 経路計画完了: {len(self.waypoints)}ウェイポイント")
        self.preflight_results['path_validity'] = True
        return passed

    # ============================================================================
    # GPS喪失ホバリングモード
    # ============================================================================

    def detect_gps_loss(self) -> bool:
        """GPS喪失検出"""
        return (self.telemetry.gps_fix_type < 3 or
                self.telemetry.gps_satellites < 10)

    def enter_gps_loss_hovering(self):
        """GPS喪失ホバリングモード開始"""
        logger.warning("=" * 70)
        logger.warning("GPS信号喪失検出 - ホバリングモードに移行")
        logger.warning("=" * 70)

        # 最終有効位置・高度を記録
        self.gps_loss.loss_time = time.time()
        self.gps_loss.last_valid_position = self.telemetry.local_position
        self.gps_loss.last_valid_altitude = self.telemetry.relative_alt
        self.gps_loss.is_active = True

        logger.info(f"最終有効位置: {self.gps_loss.last_valid_position}")
        logger.info(f"最終有効高度: {self.gps_loss.last_valid_altitude:.1f}m")
        logger.info(f"30秒カウントダウン開始")

        # 状態遷移
        self.previous_state = self.state
        self.state = AutonomyState.HOVERING

    def hovering_control(self):
        """ホバリング制御 (GPS喪失時)"""

        # 固定位置・ゼロ速度を送信 (20Hz)
        fixed_position = np.array(self.gps_loss.last_valid_position)
        zero_velocity = np.zeros(3)

        self.send_position_velocity_command(fixed_position, zero_velocity)

    def monitor_hovering_anomalies(self) -> Tuple[bool, str]:
        """
        ホバリング中の異常監視

        Returns:
            (anomaly_detected, error_message)
        """

        # EKF速度推定値チェック
        vx, vy, vz = self.telemetry.local_velocity
        velocity_norm = np.sqrt(vx**2 + vy**2 + vz**2)
        if velocity_norm > 5.0:
            return True, f"EKF速度異常 (現在{velocity_norm:.1f}m/s、許容5m/s以下)"

        # 高度維持チェック
        alt_deviation = abs(self.telemetry.relative_alt - self.gps_loss.last_valid_altitude)
        if alt_deviation > 2.0:
            return True, f"高度維持失敗 (変動{alt_deviation:.1f}m、許容±2m)"

        # 姿勢安定性チェック
        roll_deg = np.degrees(self.telemetry.attitude[0])
        pitch_deg = np.degrees(self.telemetry.attitude[1])
        if abs(roll_deg) > 15 or abs(pitch_deg) > 15:
            return True, f"姿勢不安定 (ロール{roll_deg:.1f}°, ピッチ{pitch_deg:.1f}°、許容15°以下)"

        # EKFヘルスチェック
        gate_size = 5.0
        vel_ratio = np.sqrt(self.telemetry.ekf_velocity_variance) / gate_size
        pos_ratio = np.sqrt(self.telemetry.ekf_pos_horiz_variance) / gate_size

        if vel_ratio >= 1.0 or pos_ratio >= 1.0:
            return True, f"EKFヘルス異常 (vel_ratio={vel_ratio:.2f}, pos_ratio={pos_ratio:.2f})"

        return False, ""

    def check_gps_recovery(self) -> bool:
        """GPS回復確認"""
        return (self.telemetry.gps_fix_type >= 3 and
                self.telemetry.gps_satellites >= 10)

    # ============================================================================
    # 通信ロス検出
    # ============================================================================

    def detect_comm_loss(self) -> bool:
        """通信ロス検出 (5秒タイムアウト - 新仕様)"""
        return (time.time() - self.telemetry.last_heartbeat) > self.heartbeat_timeout

    # ============================================================================
    # 障害物回避モード管理
    # ============================================================================

    def enter_avoiding_mode(self, avoidance_direction: np.ndarray):
        """障害物回避モードに移行"""
        logger.warning("=" * 70)
        logger.warning("CRITICAL障害物検出 - 回避モードに移行")
        logger.warning("=" * 70)

        self.avoidance.is_active = True
        self.avoidance.avoidance_start_time = time.time()
        self.avoidance.original_waypoint_index = self.current_wp_index
        self.avoidance.avoidance_direction = avoidance_direction

        self.previous_state = self.state
        self.state = AutonomyState.AVOIDING

    def exit_avoiding_mode(self):
        """障害物回避モード終了"""
        logger.info("=" * 70)
        logger.info("障害物回避完了 - 通常飛行に復帰")
        logger.info("=" * 70)

        self.avoidance.is_active = False
        self.state = AutonomyState.FLYING

    def should_trigger_replanning(self) -> bool:
        """経路再計画をトリガーすべきか判定"""
        # 回避モードが10秒以上継続している場合
        if self.avoidance.is_active:
            elapsed = time.time() - self.avoidance.avoidance_start_time
            if elapsed > 10.0:
                return True
        return False

    # ============================================================================
    # 状態遷移ロジック
    # ============================================================================

    def update_state(self):
        """状態更新 (10Hz周期で呼び出される)"""

        # テレメトリ受信
        self.receive_telemetry()

        # ハートビート送信 (1Hz)
        self.send_heartbeat()

        # 状態遷移処理
        if self.state == AutonomyState.INIT:
            self._handle_init()

        elif self.state == AutonomyState.READY:
            self._handle_ready()

        elif self.state == AutonomyState.FLYING:
            self._handle_flying()

        elif self.state == AutonomyState.AVOIDING:
            self._handle_avoiding()

        elif self.state == AutonomyState.REPLANNING:
            self._handle_replanning()

        elif self.state == AutonomyState.HOVERING:
            self._handle_hovering()

        elif self.state == AutonomyState.LANDING:
            self._handle_landing()

        elif self.state == AutonomyState.ERROR:
            self._handle_error()

    def _handle_init(self):
        """① INIT: NFZ取得 + 経路計画"""

        # NFZデータ取得
        if not self.nfz_loaded:
            logger.info("NFZデータ取得中...")
            self.flight.update_no_fly_zones_async()
            # 更新完了を待機
            if not self.flight.nfz_updating:
                self.nfz_loaded = True
                logger.info(f"NFZデータ取得完了: {len(self.flight.no_fly_zones)}区域")

        # ミッション受信
        if not self.mission_ready:
            self.check_mission_ready_parameter()
            if self.mission_ready:
                self.receive_mission_items()

        # 経路計画
        if self.nfz_loaded and self.mission_ready and len(self.mission_items) > 0 and not self.receiving_mission:
            if not self.path_planned:
                logger.info("AI経路最適化実行中...")
                success = self._optimize_route()
                if success:
                    self.path_planned = True
                    self.init_complete = True
                    logger.info("初期化完了 - READY状態へ移行")
                    self._transition_state(AutonomyState.READY)
                else:
                    logger.error("経路最適化失敗")
                    self._transition_state(AutonomyState.ERROR)

    def _handle_ready(self):
        """② READY: プリフライトチェック + アーム + 離陸"""

        # プリフライトチェック実行
        if self.run_preflight_checks():
            # アームコマンド送信
            self.send_arm_command()
            time.sleep(2)  # アーム完了待機

            # 離陸コマンド送信
            self.send_takeoff_command(self.takeoff_altitude)
            time.sleep(2)  # 離陸開始待機

            # FLYING状態へ移行
            logger.info("離陸開始 - FLYING状態へ移行")
            self._transition_state(AutonomyState.FLYING)
        else:
            # プリフライトチェック失敗
            logger.error("プリフライトチェック失敗 - ERROR状態へ移行")
            self._transition_state(AutonomyState.ERROR)

    def _handle_flying(self):
        """③ FLYING: 自律飛行実行"""

        # 通信ロスチェック
        if self.detect_comm_loss():
            logger.warning("通信ロス検出 - LANDING状態へ移行")
            self._transition_state(AutonomyState.LANDING)
            return

        # GPS喪失チェック
        if self.detect_gps_loss():
            self.enter_gps_loss_hovering()
            return

        # ミッション完了チェック
        if self.current_wp_index >= len(self.waypoints):
            logger.info("全ウェイポイント到達 - LANDING状態へ移行")
            self._transition_state(AutonomyState.LANDING)
            return

        # 障害物回避モードへの移行は obstacle_avoidance_loop から呼び出される

    def _handle_avoiding(self):
        """④ AVOIDING: 障害物回避中"""

        # 通信ロスチェック
        if self.detect_comm_loss():
            logger.warning("通信ロス検出 - LANDING状態へ移行")
            self._transition_state(AutonomyState.LANDING)
            return

        # GPS喪失チェック
        if self.detect_gps_loss():
            self.enter_gps_loss_hovering()
            return

        # 経路再計画が必要か判定
        if self.should_trigger_replanning():
            logger.info("回避継続中 - REPLANNING状態へ移行")
            self._transition_state(AutonomyState.REPLANNING)
            return

        # 回避完了は obstacle_avoidance_loop から exit_avoiding_mode() 呼び出しで処理

    def _handle_replanning(self):
        """⑤ REPLANNING: 経路再計算中"""

        logger.info("経路再計算実行中...")

        # 現在位置から目的地までの経路を再計算
        current_pos = self.flight.current_position
        destination = self.waypoints[-1] if len(self.waypoints) > 0 else None

        if destination is not None:
            # 簡易再計画: 現在のウェイポイントリストを維持し、現在位置からリスタート
            self.current_wp_index = min(self.current_wp_index, len(self.waypoints) - 1)
            logger.info(f"経路再計算完了: WP{self.current_wp_index}から再開")
            self._transition_state(AutonomyState.FLYING)
        else:
            logger.error("経路再計算失敗 - 目的地不明")
            self._transition_state(AutonomyState.ERROR)

    def _handle_hovering(self):
        """⑥ HOVERING: GPS喪失ホバリング"""

        # ホバリング制御実行
        self.hovering_control()

        # 異常監視
        anomaly_detected, error_msg = self.monitor_hovering_anomalies()
        if anomaly_detected:
            logger.error(f"ホバリング中異常検出: {error_msg}")
            logger.warning("LANDING状態へ移行")
            self.gps_loss.is_active = False
            self._transition_state(AutonomyState.LANDING)
            return

        # GPS回復チェック
        if self.check_gps_recovery():
            elapsed = time.time() - self.gps_loss.loss_time
            logger.info(f"GPS回復確認 ({elapsed:.1f}秒後) - 元の状態へ復帰")
            self.gps_loss.is_active = False
            self._transition_state(self.previous_state if self.previous_state != AutonomyState.HOVERING else AutonomyState.FLYING)
            return

        # タイムアウトチェック
        elapsed = time.time() - self.gps_loss.loss_time
        if elapsed > self.gps_loss.timeout_duration:
            logger.warning(f"GPS未回復 ({elapsed:.1f}秒経過) - LANDING状態へ移行")
            self.gps_loss.is_active = False
            self._transition_state(AutonomyState.LANDING)

    def _handle_landing(self):
        """⑦ LANDING: 着陸シーケンス"""

        # 着陸コマンド送信 (初回のみ)
        if self.previous_state != AutonomyState.LANDING:
            self.send_land_command()

        # 着陸完了確認
        if self.telemetry.relative_alt < 0.5:  # 高度0.5m未満
            logger.info("着陸完了 - ディスアーム実行")
            self.send_disarm_command()
            time.sleep(2)

            # STABILIZEモードへ移行
            self.send_set_mode('STABILIZE')
            logger.info("自律飛行シーケンス完了")

            # INIT状態へリセット
            self.init_complete = False
            self.nfz_loaded = False
            self.path_planned = False
            self.mission_ready = False
            self._transition_state(AutonomyState.INIT)

    def _handle_error(self):
        """⑧ ERROR: エラー状態 (統合エラー)"""

        logger.error("ERROR状態: 自動着陸を実行")

        # 自動的にLANDING状態へ移行
        self._transition_state(AutonomyState.LANDING)

    def _optimize_route(self) -> bool:
        """ミッションアイテムからAI最適化ルート生成"""
        try:
            if not self.mission_items:
                logger.error("ミッションアイテムなし")
                return False

            # 現在位置
            current_pos = self.flight.current_position

            # 目的地・中間ウェイポイント抽出
            destination_item = self.mission_items[-1]
            waypoint_items = self.mission_items[:-1] if len(self.mission_items) > 1 else []

            logger.info(f"AI最適化実行: 目的地1点 + ウェイポイント{len(waypoint_items)}点")

            # AI最適化
            optimized_waypoints = self.route_optimizer.optimize_route(
                start_pos=current_pos,
                destination=destination_item,
                waypoints=waypoint_items,
                obstacles=None
            )

            # ウェイポイント設定
            self.waypoints = optimized_waypoints
            self.current_wp_index = 0

            # FlightControllerにも設定 (NFZチェック含む)
            success = self.flight.set_waypoints([tuple(wp) for wp in self.waypoints])

            if success:
                logger.info(f"AI最適化完了: {len(self.waypoints)}ウェイポイント")
            else:
                logger.error("ウェイポイント設定失敗 (NFZ違反の可能性)")

            return success

        except Exception as e:
            logger.error(f"ルート最適化エラー: {e}", exc_info=True)
            return False

    def _transition_state(self, new_state: AutonomyState):
        """状態遷移"""
        logger.info(f"状態遷移: {self.state.name} → {new_state.name}")
        self.previous_state = self.state
        self.state = new_state

    # ============================================================================
    # ユーティリティ
    # ============================================================================

    def get_state_info(self) -> dict:
        """現在の状態情報を取得"""
        return {
            'state': self.state.name,
            'state_value': self.state.value,
            'previous_state': self.previous_state.name if self.previous_state else None,
            'mission_ready': self.mission_ready,
            'mission_count': len(self.mission_items),
            'waypoint_count': len(self.waypoints),
            'current_waypoint': self.current_wp_index,
            'gps_loss_active': self.gps_loss.is_active,
            'avoidance_active': self.avoidance.is_active,
            'init_complete': self.init_complete,
            'telemetry': {
                'gps_satellites': self.telemetry.gps_satellites,
                'gps_hdop': self.telemetry.gps_hdop,
                'battery_remaining': self.telemetry.battery_remaining,
                'relative_alt': self.telemetry.relative_alt,
                'last_heartbeat_age': time.time() - self.telemetry.last_heartbeat
            }
        }

    def is_autonomous_active(self) -> bool:
        """自律飛行がアクティブかどうか"""
        return self.state in [AutonomyState.FLYING, AutonomyState.AVOIDING, AutonomyState.REPLANNING]

    def get_current_state(self) -> AutonomyState:
        """現在の状態を取得"""
        return self.state
