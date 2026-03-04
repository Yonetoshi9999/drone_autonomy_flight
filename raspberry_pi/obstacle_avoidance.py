"""
障害物検知・回避モジュール (新仕様 v2.0)
LiDARとカメラの統合処理 + 50Hz高速障害物回避

新機能:
- 50Hz動作サイクル (20ms周期)
- 動的距離しきい値 (速度ベース)
- TTC (Time-to-Collision) 評価
- 方向優先度付き回避 (上方 > 側方 > 下方)
- センサーバッファリングと時系列追跡
- 回避完了条件の3段階チェック
"""

import numpy as np
import cv2
from collections import deque
import time
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
from enum import Enum

class CollisionRisk(Enum):
    """衝突リスクレベル"""
    SAFE = 0
    WARNING = 1
    CRITICAL = 2

class AvoidanceDirection(Enum):
    """回避方向"""
    UPWARD = 0      # 上昇
    LATERAL = 1     # 側方
    DOWNWARD = 2    # 下降
    NONE = 3        # 回避不要

@dataclass
class ObstacleInfo:
    """障害物情報"""
    position: np.ndarray  # 3D位置 (NED)
    velocity: np.ndarray  # 3D速度 (NED)
    confidence: float  # 信頼度 (0.0-1.0)
    distance: float  # ドローンからの距離 (m)
    ttc: float  # Time-to-Collision (秒)
    d_cpa: float  # Distance at Closest Point of Approach (m)
    risk_level: CollisionRisk  # リスクレベル
    timestamp: float  # タイムスタンプ
    sensor_type: str  # センサータイプ ('lidar', 'camera', 'fused')

class ObstacleAvoidance:
    """障害物検知・回避クラス (新仕様 v2.0)"""

    def __init__(self):
        # 検知パラメータ (動的に計算)
        self.base_detection_range = 10.0  # m
        self.safety_margin = 3.0     # m

        # 動的しきい値パラメータ
        self.critical_base = 2.0  # m
        self.critical_speed_factor = 0.3  # s
        self.warning_base = 5.0  # m
        self.warning_speed_factor = 0.5  # s

        # TTC しきい値
        self.ttc_critical = 2.0  # s
        self.ttc_warning = 5.0  # s
        self.d_cpa_threshold = 3.0  # m

        # センサーバッファ
        self.lidar_buffer = deque(maxlen=10)  # 200ms履歴 (50Hz × 10)
        self.camera_buffer = deque(maxlen=5)  # 500ms履歴 (10Hz × 5)
        self.obstacle_history = deque(maxlen=50)  # 1秒履歴 (50Hz × 50)

        # 速度履歴 (移動平均用)
        self.velocity_history = deque(maxlen=5)

        # 回避完了判定パラメータ
        self.avoidance_clearance_distance = 10.0  # m
        self.return_path_sample_distance = 1.0  # m
        self.return_path_lookahead = 30.0  # m
        self.return_path_clearance = 5.0  # m

        # OpenCV設定
        cv2.setUseOptimized(True)
        cv2.setNumThreads(2)

        # カメラパラメータ（キャリブレーション済み）
        self.camera_matrix = np.array([
            [921.6, 0, 320],
            [0, 921.6, 240],
            [0, 0, 1]
        ])

        # 回避状態
        self.is_avoiding = False
        self.avoidance_start_time = 0.0

    def detect_and_assess(self, lidar_data, camera_frame, drone_position: np.ndarray,
                          drone_velocity: np.ndarray) -> Tuple[List[ObstacleInfo], CollisionRisk]:
        """
        統合障害物検知・評価 (50Hz / 20ms周期)

        Args:
            lidar_data: LiDARスキャンデータ
            camera_frame: カメラ画像
            drone_position: ドローン現在位置 (NED)
            drone_velocity: ドローン速度 (NED)

        Returns:
            (obstacles, max_risk_level)
        """
        start_time = time.perf_counter()

        # 速度履歴更新
        self.velocity_history.append(drone_velocity.copy())
        avg_velocity = np.mean(list(self.velocity_history), axis=0)

        # 現在速度からしきい値を計算
        speed = np.linalg.norm(avg_velocity)
        critical_distance = self.critical_base + speed * self.critical_speed_factor
        warning_distance = self.warning_base + speed * self.warning_speed_factor

        obstacles = []

        # 1. LiDAR処理 (8ms)
        if lidar_data:
            lidar_obstacles = self.process_lidar_buffered(lidar_data, drone_position, avg_velocity)
            obstacles.extend(lidar_obstacles)

        # 2. カメラ処理 (7ms)
        if camera_frame is not None:
            camera_obstacles = self.process_camera_buffered(camera_frame, drone_position, avg_velocity)
            obstacles.extend(camera_obstacles)

        # 3. データ融合 (2ms)
        fused_obstacles = self.fuse_sensor_data(obstacles)

        # 4. TTC評価 (2ms)
        assessed_obstacles = self.assess_collision_risk(
            fused_obstacles,
            drone_position,
            avg_velocity,
            critical_distance,
            warning_distance
        )

        # 履歴更新
        self.obstacle_history.append(assessed_obstacles)

        # 最大リスクレベル決定
        max_risk = CollisionRisk.SAFE
        for obs in assessed_obstacles:
            if obs.risk_level.value > max_risk.value:
                max_risk = obs.risk_level

        elapsed = (time.perf_counter() - start_time) * 1000
        if elapsed > 19:  # 19ms警告（20ms予算 - 1ms余裕）
            print(f"警告: 障害物検知時間超過: {elapsed:.1f}ms")

        return assessed_obstacles, max_risk

    def process_lidar_buffered(self, scan_data, drone_position: np.ndarray,
                                drone_velocity: np.ndarray) -> List[ObstacleInfo]:
        """LiDARデータ処理 (バッファリング付き)"""
        timestamp = time.time()

        # バッファに追加
        self.lidar_buffer.append({
            'data': scan_data,
            'timestamp': timestamp,
            'drone_position': drone_position.copy(),
            'drone_velocity': drone_velocity.copy()
        })

        obstacles = []

        # 簡易クラスタリング
        clusters = []
        current_cluster = []

        for quality, angle, distance in scan_data:
            if quality < 10:  # 品質フィルタ
                continue

            if distance > self.base_detection_range * 1000:  # mm変換
                continue

            # 極座標から直交座標へ (ドローン基準)
            x = distance * np.cos(np.radians(angle)) / 1000
            y = distance * np.sin(np.radians(angle)) / 1000
            z = 0  # LiDARは水平面スキャン

            # ワールド座標に変換
            obstacle_local = np.array([x, y, z])
            obstacle_world = drone_position + obstacle_local

            # クラスタリング（連続した点を障害物として認識）
            if current_cluster:
                last_point = current_cluster[-1]
                dist_diff = abs(distance - last_point[3])

                if dist_diff < 200:  # 200mm以内は同じ障害物
                    current_cluster.append((x, y, z, distance, obstacle_world))
                else:
                    if len(current_cluster) > 3:
                        clusters.append(current_cluster)
                    current_cluster = [(x, y, z, distance, obstacle_world)]
            else:
                current_cluster = [(x, y, z, distance, obstacle_world)]

        # 最後のクラスタを追加
        if len(current_cluster) > 3:
            clusters.append(current_cluster)

        # クラスタを障害物に変換
        for cluster in clusters:
            positions = np.array([p[4] for p in cluster])
            center = np.mean(positions, axis=0)
            distance = np.linalg.norm(center - drone_position)

            # 速度推定 (過去フレームとの比較)
            velocity = self._estimate_obstacle_velocity(center, timestamp)

            obstacles.append(ObstacleInfo(
                position=center,
                velocity=velocity,
                confidence=min(1.0, len(cluster) / 10),
                distance=distance,
                ttc=0.0,  # 後で計算
                d_cpa=0.0,  # 後で計算
                risk_level=CollisionRisk.SAFE,  # 後で計算
                timestamp=timestamp,
                sensor_type='lidar'
            ))

        return obstacles

    def process_camera_buffered(self, frame, drone_position: np.ndarray,
                                 drone_velocity: np.ndarray) -> List[ObstacleInfo]:
        """カメラ画像処理 (バッファリング付き)"""
        timestamp = time.time()

        # バッファに追加
        self.camera_buffer.append({
            'frame': frame.copy(),
            'timestamp': timestamp,
            'drone_position': drone_position.copy(),
            'drone_velocity': drone_velocity.copy()
        })

        obstacles = []

        # 解像度削減（高速化）
        small_frame = cv2.resize(frame, (320, 240))
        gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)

        # 簡易エッジ検出
        edges = cv2.Canny(gray, 50, 100, apertureSize=3, L2gradient=False)

        # 輪郭検出（最大10個）
        contours, _ = cv2.findContours(
            edges,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # 面積でソートして上位を処理
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:10]

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 100:  # 小さすぎる輪郭は無視
                continue

            # バウンディングボックス
            x, y, w, h = cv2.boundingRect(contour)

            # 簡易距離推定（画像下部ほど近い）
            estimated_distance = max(1.0, 10.0 * (240 - (y + h/2)) / 240)

            # 3D位置推定（単純化）
            cx = x + w/2 - 160  # 画像中心からのオフセット
            position_x = cx * estimated_distance / 320
            position_y = 0  # カメラ正面方向
            position_z = estimated_distance

            # ワールド座標
            obstacle_local = np.array([position_z, position_x, 0])
            obstacle_world = drone_position + obstacle_local
            distance = np.linalg.norm(obstacle_world - drone_position)

            # 速度推定
            velocity = self._estimate_obstacle_velocity(obstacle_world, timestamp)

            obstacles.append(ObstacleInfo(
                position=obstacle_world,
                velocity=velocity,
                confidence=min(1.0, area / 1000),
                distance=distance,
                ttc=0.0,
                d_cpa=0.0,
                risk_level=CollisionRisk.SAFE,
                timestamp=timestamp,
                sensor_type='camera'
            ))

        return obstacles

    def _estimate_obstacle_velocity(self, position: np.ndarray, timestamp: float) -> np.ndarray:
        """障害物速度推定 (過去フレームとの比較)"""
        # 過去の障害物から近い位置のものを探す
        if len(self.obstacle_history) < 2:
            return np.zeros(3)

        prev_obstacles = self.obstacle_history[-2] if len(self.obstacle_history) >= 2 else []

        # 最も近い過去の障害物を探す
        min_dist = float('inf')
        closest_obs = None

        for obs in prev_obstacles:
            dist = np.linalg.norm(obs.position - position)
            if dist < min_dist and dist < 2.0:  # 2m以内で同一障害物と判定
                min_dist = dist
                closest_obs = obs

        if closest_obs is not None:
            dt = timestamp - closest_obs.timestamp
            if dt > 0:
                velocity = (position - closest_obs.position) / dt
                return velocity

        return np.zeros(3)

    def fuse_sensor_data(self, obstacles: List[ObstacleInfo]) -> List[ObstacleInfo]:
        """センサーデータ融合"""

        if not obstacles:
            return []

        # 重複除去と信頼度統合
        fused = []
        used = [False] * len(obstacles)

        for i, obs1 in enumerate(obstacles):
            if used[i]:
                continue

            combined = obs1
            count = 1

            # 近い障害物を統合
            for j, obs2 in enumerate(obstacles[i+1:], i+1):
                if used[j]:
                    continue

                dist = np.linalg.norm(obs1.position - obs2.position)

                if dist < 1.0:  # 1m以内は同じ障害物
                    # 位置を加重平均
                    w1 = combined.confidence
                    w2 = obs2.confidence
                    combined.position = (
                        combined.position * w1 +
                        obs2.position * w2
                    ) / (w1 + w2)

                    # 速度も加重平均
                    combined.velocity = (
                        combined.velocity * w1 +
                        obs2.velocity * w2
                    ) / (w1 + w2)

                    combined.confidence = min(1.0, w1 + w2)
                    combined.sensor_type = 'fused'
                    used[j] = True
                    count += 1

            # マルチセンサーで検出された障害物は信頼度UP
            if count > 1:
                combined.confidence = min(1.0, combined.confidence * 1.5)

            fused.append(combined)

        return fused

    def assess_collision_risk(self, obstacles: List[ObstacleInfo], drone_position: np.ndarray,
                               drone_velocity: np.ndarray, critical_distance: float,
                               warning_distance: float) -> List[ObstacleInfo]:
        """衝突リスク評価 (TTC計算)"""

        assessed = []

        for obs in obstacles:
            # 相対位置・速度
            r = obs.position - drone_position
            v = obs.velocity - drone_velocity

            # TTC (Time to Collision) 計算
            v_norm_sq = np.dot(v, v)

            if v_norm_sq > 1e-6:  # ゼロ除算回避
                # Closest Point of Approach (CPA) までの時間
                t_cpa = -np.dot(r, v) / v_norm_sq

                # CPA時の距離
                if t_cpa > 0:
                    r_cpa = r + v * t_cpa
                    d_cpa = np.linalg.norm(r_cpa)
                else:
                    d_cpa = obs.distance
                    t_cpa = float('inf')
            else:
                t_cpa = float('inf')
                d_cpa = obs.distance

            obs.ttc = t_cpa
            obs.d_cpa = d_cpa

            # リスクレベル決定
            if obs.distance < critical_distance or (t_cpa < self.ttc_critical and d_cpa < self.d_cpa_threshold):
                obs.risk_level = CollisionRisk.CRITICAL
            elif obs.distance < warning_distance or (t_cpa < self.ttc_warning and d_cpa < self.d_cpa_threshold):
                obs.risk_level = CollisionRisk.WARNING
            else:
                obs.risk_level = CollisionRisk.SAFE

            assessed.append(obs)

        return assessed

    def select_avoidance_direction(self, obstacles: List[ObstacleInfo], drone_position: np.ndarray,
                                     drone_altitude: float, lidar_data) -> Tuple[AvoidanceDirection, np.ndarray]:
        """
        回避方向選択 (優先度: 上方 > 側方 > 下方)

        Returns:
            (direction_type, direction_vector)
        """

        # 1. 上方回避チェック (最優先)
        if drone_altitude < 140.0:  # 高度制限150m以下
            # 上方15m以内に障害物がないかチェック
            upward_clear = True
            for obs in obstacles:
                relative_pos = obs.position - drone_position
                if relative_pos[2] < 0 and abs(relative_pos[2]) < 15.0:  # NEDなので-Z が上方
                    if np.linalg.norm(relative_pos[:2]) < 5.0:  # 水平5m以内
                        upward_clear = False
                        break

            if upward_clear:
                return AvoidanceDirection.UPWARD, np.array([0, 0, -1.0])  # NED: -Z = 上昇

        # 2. 側方回避チェック (360°スキャン)
        if lidar_data:
            best_direction, best_score = self._find_best_lateral_direction(
                lidar_data, obstacles, drone_position
            )

            if best_score > 0.5:  # スコアが十分高い
                return AvoidanceDirection.LATERAL, best_direction

        # 3. 下方回避チェック (最終手段)
        if drone_altitude > 20.0:  # 地上20m以上
            # 下方10m以内に障害物がないかチェック
            downward_clear = True
            for obs in obstacles:
                relative_pos = obs.position - drone_position
                if relative_pos[2] > 0 and relative_pos[2] < 10.0:  # NED: +Z = 下降
                    if np.linalg.norm(relative_pos[:2]) < 5.0:
                        downward_clear = False
                        break

            if downward_clear:
                return AvoidanceDirection.DOWNWARD, np.array([0, 0, 1.0])  # NED: +Z = 下降

        # 回避方向なし（緊急停止）
        return AvoidanceDirection.NONE, np.zeros(3)

    def _find_best_lateral_direction(self, lidar_data, obstacles: List[ObstacleInfo],
                                      drone_position: np.ndarray) -> Tuple[np.ndarray, float]:
        """最適な側方回避方向を探索 (360°スキャン、15°刻み)"""

        best_direction = np.array([1.0, 0, 0])
        best_score = 0.0

        # 15°刻みで24方向をチェック
        for i in range(24):
            angle = i * 15.0  # 度
            angle_rad = np.radians(angle)

            # 方向ベクトル (NED: X=北, Y=東)
            direction = np.array([
                np.cos(angle_rad),
                np.sin(angle_rad),
                0
            ])

            # この方向のスコア計算
            score = self._evaluate_direction_score(direction, lidar_data, obstacles, drone_position)

            if score > best_score:
                best_score = score
                best_direction = direction

        return best_direction, best_score

    def _evaluate_direction_score(self, direction: np.ndarray, lidar_data,
                                   obstacles: List[ObstacleInfo], drone_position: np.ndarray) -> float:
        """方向スコア評価 (0.0-1.0)"""

        score = 1.0

        # 障害物との距離を評価
        for obs in obstacles:
            relative_pos = obs.position - drone_position

            # 方向との内積（正なら前方、負なら後方）
            dot_product = np.dot(relative_pos, direction)

            if dot_product > 0:  # 前方にある障害物
                # 距離が近いほどペナルティ
                distance = np.linalg.norm(relative_pos)
                penalty = max(0, 1.0 - distance / 10.0)
                score -= penalty * 0.3

        # LiDARデータで詳細チェック
        if lidar_data:
            direction_angle = np.degrees(np.arctan2(direction[1], direction[0]))

            # ±30°範囲をチェック
            for quality, angle, distance in lidar_data:
                angle_diff = abs(angle - direction_angle)
                if angle_diff > 180:
                    angle_diff = 360 - angle_diff

                if angle_diff < 30:  # ±30°範囲内
                    dist_m = distance / 1000.0
                    if dist_m < 5.0:
                        penalty = max(0, 1.0 - dist_m / 5.0)
                        score -= penalty * 0.2

        return max(0.0, score)

    def check_avoidance_completion(self, obstacles: List[ObstacleInfo], drone_position: np.ndarray,
                                     drone_velocity: np.ndarray, global_path: List[np.ndarray],
                                     current_wp_index: int) -> bool:
        """
        回避完了判定 (3条件チェック)

        Returns:
            bool: 回避完了時True
        """

        # 条件1: 全障害物との距離 ≥ 10m
        min_distance = float('inf')
        for obs in obstacles:
            if obs.risk_level != CollisionRisk.SAFE:
                min_distance = min(min_distance, obs.distance)

        if min_distance < self.avoidance_clearance_distance:
            return False

        # 条件2: 相対速度が分離方向
        for obs in obstacles:
            if obs.risk_level != CollisionRisk.SAFE:
                relative_velocity = obs.velocity - drone_velocity
                relative_position = obs.position - drone_position

                # 内積が負なら分離中
                dot_product = np.dot(relative_velocity, relative_position)
                if dot_product >= 0:
                    return False

        # 条件3: グローバル経路への復帰パスがクリア
        if not self._check_return_path_clear(drone_position, global_path, current_wp_index, obstacles):
            return False

        return True

    def _check_return_path_clear(self, drone_position: np.ndarray, global_path: List[np.ndarray],
                                   current_wp_index: int, obstacles: List[ObstacleInfo]) -> bool:
        """グローバル経路への復帰パスチェック"""

        if current_wp_index >= len(global_path):
            return True

        # 30m先のウェイポイントを探す
        target_wp = None
        for i in range(current_wp_index, len(global_path)):
            wp = np.array(global_path[i])
            distance = np.linalg.norm(wp - drone_position)

            if distance >= self.return_path_lookahead:
                target_wp = wp
                break

        if target_wp is None:
            if len(global_path) > current_wp_index:
                target_wp = np.array(global_path[-1])  # 最終ウェイポイント
            else:
                return True

        # 復帰経路を1mサンプリングでチェック
        direction = target_wp - drone_position
        distance = np.linalg.norm(direction)

        if distance < 0.1:
            return True

        direction = direction / distance
        num_samples = int(distance / self.return_path_sample_distance)

        for i in range(num_samples):
            sample_pos = drone_position + direction * (i * self.return_path_sample_distance)

            # この位置から障害物までの最小距離をチェック
            for obs in obstacles:
                obs_distance = np.linalg.norm(obs.position - sample_pos)
                if obs_distance < self.return_path_clearance:
                    return False

        return True

    def get_avoidance_velocity(self, direction: AvoidanceDirection, direction_vector: np.ndarray,
                                max_speed: float = 3.0) -> np.ndarray:
        """回避速度ベクトル生成"""

        if direction == AvoidanceDirection.NONE:
            return np.zeros(3)

        # 方向ベクトルを正規化して速度スケール
        norm = np.linalg.norm(direction_vector)
        if norm > 0:
            return direction_vector / norm * max_speed
        else:
            return np.zeros(3)
