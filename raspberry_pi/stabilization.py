"""
ジンバル制御モジュール
カメラ安定化
"""

import numpy as np
from pymavlink import mavutil
import time

class GimbalStabilization:
    """ジンバル安定化クラス"""
    
    def __init__(self):
        # ジンバル設定
        self.pitch_range = (-90, 30)  # 度
        self.roll_range = (-30, 30)
        self.yaw_range = (-180, 180)
        
        # 現在角度
        self.current_pitch = 0
        self.current_roll = 0
        self.current_yaw = 0
        
        # 制御モード
        self.mode = 'stabilized'  # 'stabilized', 'lock', 'follow'
        
    def update(self, target_angles=None, drone_attitude=None):
        """ジンバル更新"""
        
        if self.mode == 'stabilized':
            # 姿勢安定化モード
            self.stabilize(drone_attitude)
            
        elif self.mode == 'lock':
            # 地球固定モード
            self.lock_position(target_angles)
            
        elif self.mode == 'follow':
            # 機体追従モード
            self.follow_mode()
    
    def stabilize(self, drone_attitude):
        """姿勢安定化"""
        
        if drone_attitude is None:
            return
        
        # ドローンの姿勢を補正
        compensation_roll = -drone_attitude['roll']
        compensation_pitch = -drone_attitude['pitch']
        
        # ジンバル角度を設定
        self.set_gimbal_angle(
            pitch=self.current_pitch + compensation_pitch,
            roll=compensation_roll
        )
    
    def lock_position(self, target_angles):
        """位置固定"""
        
        if target_angles:
            self.set_gimbal_angle(
                pitch=target_angles.get('pitch', self.current_pitch),
                roll=target_angles.get('roll', self.current_roll),
                yaw=target_angles.get('yaw', self.current_yaw)
            )
    
    def follow_mode(self):
        """機体追従モード"""
        # 機体の向きに追従
        pass
    
    def set_gimbal_angle(self, pitch=None, roll=None, yaw=None):
        """ジンバル角度設定"""
        
        if pitch is not None:
            self.current_pitch = np.clip(pitch, *self.pitch_range)
            
        if roll is not None:
            self.current_roll = np.clip(roll, *self.roll_range)
            
        if yaw is not None:
            self.current_yaw = np.clip(yaw, *self.yaw_range)
        
        # PWM信号送信（実際のハードウェア制御）
        self.send_pwm_signal()
    
    def send_pwm_signal(self):
        """PWM信号送信"""
        # 実際のジンバルハードウェアにPWM送信
        # ここではシミュレーション
        pass
    
    def point_at_target(self, target_position, drone_position):
        """特定位置を向く"""
        
        # 相対位置計算
        diff = target_position - drone_position
        
        # 角度計算
        distance = np.linalg.norm(diff[:2])
        pitch_angle = np.degrees(np.arctan2(-diff[2], distance))
        yaw_angle = np.degrees(np.arctan2(diff[1], diff[0]))
        
        self.set_gimbal_angle(pitch=pitch_angle, yaw=yaw_angle)