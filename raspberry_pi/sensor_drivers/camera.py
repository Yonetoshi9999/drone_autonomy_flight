"""
カメラドライバー
Sony IMX477 / OV9281用
"""

import cv2
import numpy as np
from picamera2 import Picamera2
import time

class CameraDriver:
    """カメラドライバークラス"""
    
    def __init__(self, camera_type='imx477'):
        self.camera_type = camera_type
        self.camera = None
        self.is_running = False
        
        # 解像度設定
        if camera_type == 'imx477':
            self.resolution = (1920, 1080)
            self.fps = 30
        elif camera_type == 'ov9281':
            self.resolution = (1280, 800)
            self.fps = 60
        else:
            self.resolution = (640, 480)
            self.fps = 30
    
    def initialize(self):
        """初期化"""
        try:
            if self.camera_type in ['imx477', 'ov9281']:
                # Raspberry Pi Camera
                self.camera = Picamera2()
                config = self.camera.create_preview_configuration(
                    main={"size": self.resolution, "format": "RGB888"}
                )
                self.camera.configure(config)
                self.camera.start()
            else:
                # USB Camera
                self.camera = cv2.VideoCapture(0)
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
                self.camera.set(cv2.CAP_PROP_FPS, self.fps)
            
            self.is_running = True
            print(f"カメラ初期化成功: {self.camera_type}")
            return True
            
        except Exception as e:
            print(f"カメラ初期化失敗: {e}")
            return False
    
    def get_frame(self):
        """フレーム取得"""
        
        if not self.is_running:
            return None
        
        try:
            if isinstance(self.camera, Picamera2):
                frame = self.camera.capture_array()
            else:
                ret, frame = self.camera.read()
                if not ret:
                    return None
            
            return frame
            
        except Exception as e:
            print(f"フレーム取得エラー: {e}")
            return None
    
    def release(self):
        """解放"""
        if self.camera:
            if isinstance(self.camera, Picamera2):
                self.camera.stop()
            else:
                self.camera.release()
            self.is_running = False