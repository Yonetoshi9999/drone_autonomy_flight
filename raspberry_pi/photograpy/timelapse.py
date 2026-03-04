"""
タイムラプス撮影
"""

import cv2
import numpy as np
from datetime import datetime
import time

class TimelapseCapture:
    """タイムラプス撮影クラス"""
    
    def __init__(self):
        self.interval = 5.0  # 秒
        self.duration = 300  # 秒（5分）
        self.images = []
        self.is_capturing = False
        self.start_time = None
        
    def start(self, interval=5.0, duration=300):
        """撮影開始"""
        
        self.interval = interval
        self.duration = duration
        self.images = []
        self.is_capturing = True
        self.start_time = time.time()
        
        print(f"タイムラプス開始: {interval}秒間隔, {duration}秒間")
        
    def capture_frame(self, frame):
        """フレーム追加"""
        
        if not self.is_capturing:
            return False
        
        # 時間チェック
        elapsed = time.time() - self.start_time
        
        if elapsed > self.duration:
            self.stop()
            return False
        
        # 画像保存
        self.images.append({
            'frame': frame.copy(),
            'timestamp': datetime.now(),
            'elapsed': elapsed
        })
        
        return True
    
    def stop(self):
        """撮影停止"""
        self.is_capturing = False
        print(f"タイムラプス終了: {len(self.images)}枚撮影")
        
    def create_video(self, output_path, fps=30):
        """動画作成"""
        
        if not self.images:
            return False
        
        # 動画ライター初期化
        height, width = self.images[0]['frame'].shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        
        writer = cv2.VideoWriter(
            output_path,
            fourcc,
            fps,
            (width, height)
        )
        
        # フレーム書き込み
        for img_data in self.images:
            writer.write(img_data['frame'])
        
        writer.release()
        return True