"""
RPLidar A2M8 ドライバー
"""

from rplidar import RPLidar
import numpy as np
import time

class RPLidarDriver:
    """RPLidar A2M8ドライバークラス"""
    
    def __init__(self, port='/dev/ttyUSB0'):
        self.port = port
        self.lidar = None
        self.is_running = False
        
    def connect(self):
        """接続"""
        try:
            self.lidar = RPLidar(self.port)
            self.lidar.set_pwm(550)  # モーター速度
            info = self.lidar.get_info()
            print(f"LiDAR接続成功: {info}")
            self.is_running = True
            return True
        except Exception as e:
            print(f"LiDAR接続失敗: {e}")
            return False
    
    def get_scan(self):
        """スキャンデータ取得"""
        
        if not self.lidar or not self.is_running:
            return []
        
        try:
            scan = next(self.lidar.iter_scans(max_buf_meas=500))
            return scan
        except Exception as e:
            print(f"LiDARスキャンエラー: {e}")
            return []
    
    def stop(self):
        """停止"""
        if self.lidar:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
            self.is_running = False