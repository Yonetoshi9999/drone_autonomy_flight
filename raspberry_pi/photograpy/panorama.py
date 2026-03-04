"""
パノラマ撮影
"""

import cv2
import numpy as np
from collections import deque

class PanoramaCapture:
    """パノラマ撮影クラス"""
    
    def __init__(self):
        self.images = deque()
        self.overlap_ratio = 0.3  # 30%重複
        self.stitcher = cv2.Stitcher_create()
        
    def add_image(self, image):
        """画像追加"""
        self.images.append(image)
        
    def create_panorama(self):
        """パノラマ作成"""
        
        if len(self.images) < 2:
            return None
        
        # 画像配列に変換
        image_array = list(self.images)
        
        # スティッチング
        status, panorama = self.stitcher.stitch(image_array)
        
        if status == cv2.Stitcher_OK:
            return panorama
        else:
            print(f"パノラマ作成失敗: {status}")
            return None
    
    def calculate_rotation_angles(self, num_images=12):
        """回転角度計算"""
        
        angle_step = 360 / num_images
        angles = []
        
        for i in range(num_images):
            angles.append({
                'yaw': i * angle_step,
                'pitch': 0,
                'capture_delay': 2.0  # 秒
            })
        
        return angles
    
    def clear(self):
        """画像クリア"""
        self.images.clear()