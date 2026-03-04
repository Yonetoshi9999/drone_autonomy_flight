"""
露出制御
"""

import cv2
import numpy as np

class ExposureControl:
    """露出制御クラス"""
    
    def __init__(self):
        self.target_brightness = 128  # 目標輝度
        self.exposure_compensation = 0
        
    def analyze_exposure(self, frame):
        """露出分析"""
        
        # グレースケール変換
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # ヒストグラム計算
        hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
        
        # 統計量
        mean_brightness = np.mean(gray)
        std_brightness = np.std(gray)
        
        # 露出評価
        exposure_info = {
            'mean': mean_brightness,
            'std': std_brightness,
            'histogram': hist,
            'is_overexposed': self.check_overexposure(hist),
            'is_underexposed': self.check_underexposure(hist)
        }
        
        return exposure_info
    
    def check_overexposure(self, histogram):
        """過露出チェック"""
        # 高輝度ピクセルの割合
        bright_pixels = np.sum(histogram[240:])
        total_pixels = np.sum(histogram)
        
        return (bright_pixels / total_pixels) > 0.1
    
    def check_underexposure(self, histogram):
        """露出不足チェック"""
        # 低輝度ピクセルの割合
        dark_pixels = np.sum(histogram[:16])
        total_pixels = np.sum(histogram)
        
        return (dark_pixels / total_pixels) > 0.3
    
    def calculate_exposure_compensation(self, current_brightness):
        """露出補正値計算"""
        
        # PID制御（簡易版）
        error = self.target_brightness - current_brightness
        
        # P制御のみ
        kp = 0.01
        compensation = error * kp
        
        # 制限
        self.exposure_compensation = np.clip(
            compensation, 
            -2.0, 
            2.0
        )
        
        return self.exposure_compensation
    
    def apply_exposure_compensation(self, frame, ev_compensation):
        """露出補正適用"""
        
        # EV値を明度係数に変換
        brightness_factor = 2 ** ev_compensation
        
        # 明度調整
        adjusted = cv2.convertScaleAbs(
            frame, 
            alpha=brightness_factor, 
            beta=0
        )
        
        return adjusted