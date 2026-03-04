"""
被写体追跡
"""

import cv2
import numpy as np

class ObjectTracking:
    """被写体追跡クラス"""
    
    def __init__(self):
        # トラッカー初期化
        self.tracker = cv2.TrackerCSRT_create()
        self.is_tracking = False
        self.target_bbox = None
        
    def init_tracking(self, frame, bbox):
        """追跡開始"""
        
        self.target_bbox = bbox
        self.tracker = cv2.TrackerCSRT_create()
        self.is_tracking = self.tracker.init(frame, bbox)
        
        return self.is_tracking
    
    def update(self, frame):
        """追跡更新"""
        
        if not self.is_tracking:
            return None
        
        success, bbox = self.tracker.update(frame)
        
        if success:
            self.target_bbox = bbox
            return bbox
        else:
            self.is_tracking = False
            return None
    
    def get_target_position(self):
        """ターゲット位置取得"""
        
        if self.target_bbox:
            x, y, w, h = self.target_bbox
            center_x = x + w/2
            center_y = y + h/2
            return (center_x, center_y)
        
        return None
    
    def calculate_gimbal_command(self, target_pos, frame_center):
        """ジンバル制御コマンド計算"""
        
        if not target_pos:
            return None
        
        # 画面中心からのずれ
        dx = target_pos[0] - frame_center[0]
        dy = target_pos[1] - frame_center[1]
        
        # PID制御（簡易版）
        kp = 0.1
        
        gimbal_cmd = {
            'yaw_rate': -dx * kp,
            'pitch_rate': dy * kp
        }
        
        return gimbal_cmd