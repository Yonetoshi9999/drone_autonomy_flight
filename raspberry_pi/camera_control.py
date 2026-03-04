"""
カメラ制御モジュール
撮影と画像保存
"""

import cv2
import numpy as np
from datetime import datetime
import json
from pathlib import Path
import exifread
from PIL import Image
from PIL.ExifTags import TAGS

class CameraControl:
    """カメラ制御クラス"""
    
    def __init__(self):
        # 保存先設定
        self.base_path = Path('/home/pi/aerial_photography_drone/captured_media')
        self.photo_path = self.base_path / 'photos'
        self.video_path = self.base_path / 'videos'
        self.metadata_path = self.base_path / 'metadata'
        
        # ディレクトリ作成
        self.photo_path.mkdir(parents=True, exist_ok=True)
        self.video_path.mkdir(parents=True, exist_ok=True)
        self.metadata_path.mkdir(parents=True, exist_ok=True)
        
        # カメラパラメータ
        self.resolution = (1920, 1080)
        self.fps = 30
        self.photo_quality = 95  # JPEG品質
        
        # 撮影カウンタ
        self.photo_count = 0
        self.current_position = None
        
    def capture_and_save(self):
        """画像撮影と保存（500ms周期）"""
        
        # タイムスタンプ
        timestamp = datetime.now()
        filename_base = timestamp.strftime("%Y%m%d_%H%M%S")
        
        # 画像取得（カメラドライバー経由）
        frame = self.get_current_frame()
        
        if frame is None:
            return False
        
        # 品質チェック
        if not self.check_image_quality(frame):
            return False
        
        # ファイル保存
        photo_filename = self.photo_path / f"IMG_{filename_base}_{self.photo_count:04d}.jpg"
        
        # JPEG保存（高品質）
        cv2.imwrite(
            str(photo_filename), 
            frame, 
            [cv2.IMWRITE_JPEG_QUALITY, self.photo_quality]
        )
        
        # メタデータ保存
        metadata = self.create_metadata(timestamp)
        meta_filename = self.metadata_path / f"META_{filename_base}_{self.photo_count:04d}.json"
        
        with open(meta_filename, 'w') as f:
            json.dump(metadata, f, indent=2, default=str)
        
        self.photo_count += 1
        return True
    
    def get_current_frame(self):
        """現在のフレーム取得"""
        # カメラドライバーから取得（別モジュール）
        # ここではダミー
        return np.random.randint(0, 255, (*self.resolution[::-1], 3), dtype=np.uint8)
    
    def check_image_quality(self, frame):
        """画像品質チェック"""
        
        # ブレ検出（ラプラシアン分散）
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
        
        if laplacian_var < 100:  # ブレ閾値
            return False
        
        # 露出チェック
        hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
        
        # 極端な露出を除外
        if hist[:10].sum() > frame.size * 0.5:  # 暗すぎ
            return False
        if hist[-10:].sum() > frame.size * 0.5:  # 明るすぎ
            return False
        
        return True
    
    def create_metadata(self, timestamp):
        """撮影メタデータ作成"""
        
        metadata = {
            'timestamp': timestamp.isoformat(),
            'photo_number': self.photo_count,
            'position': self.current_position,
            'camera_settings': {
                'resolution': self.resolution,
                'quality': self.photo_quality
            },
            'drone_state': self.get_drone_state()
        }
        
        return metadata
    
    def get_drone_state(self):
        """ドローン状態取得"""
        # MAVLinkから取得（簡略化）
        return {
            'altitude': 0,
            'battery': 0,
            'gps_fix': 0
        }
    
    def start_video_recording(self):
        """動画録画開始"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        video_filename = self.video_path / f"VID_{timestamp}.mp4"
        
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.video_writer = cv2.VideoWriter(
            str(video_filename),
            fourcc,
            self.fps,
            self.resolution
        )
        
        return True
    
    def stop_video_recording(self):
        """動画録画停止"""
        if hasattr(self, 'video_writer'):
            self.video_writer.release()