"""
シーン検出
"""

import cv2
import numpy as np

class SceneDetection:
    """シーン検出クラス"""
    
    def __init__(self):
        self.scene_types = [
            'landscape',
            'urban',
            'forest',
            'water',
            'sunset'
        ]
        
    def detect_scene_type(self, frame):
        """シーンタイプ検出"""
        
        # 色分析
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 各色の割合を計算
        color_ranges = {
            'green': [(35, 50, 50), (85, 255, 255)],  # 森林
            'blue': [(100, 50, 50), (130, 255, 255)],  # 水/空
            'orange': [(10, 100, 100), (25, 255, 255)],  # 夕日
            'gray': [(0, 0, 50), (180, 30, 200)]  # 都市
        }
        
        color_ratios = {}
        total_pixels = frame.shape[0] * frame.shape[1]
        
        for color, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            ratio = np.sum(mask > 0) / total_pixels
            color_ratios[color] = ratio
        
        # シーンタイプ判定
        if color_ratios['green'] > 0.3:
            return 'forest'
        elif color_ratios['blue'] > 0.4:
            return 'water' if self.detect_water(frame) else 'landscape'
        elif color_ratios['orange'] > 0.2:
            return 'sunset'
        elif color_ratios['gray'] > 0.3:
            return 'urban'
        else:
            return 'landscape'
    
    def detect_water(self, frame):
        """水面検出"""
        
        # 簡易的な水面検出（反射パターン）
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 水平エッジ検出
        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        
        # 水平成分が強い場合は水面の可能性
        horizontal_strength = np.mean(np.abs(sobelx))
        vertical_strength = np.mean(np.abs(sobely))
        
        return horizontal_strength > vertical_strength * 1.5
    
    def adjust_camera_settings(self, scene_type):
        """シーンに応じたカメラ設定"""
        
        settings = {
            'landscape': {
                'iso': 100,
                'shutter': 1/500,
                'aperture': 8.0
            },
            'urban': {
                'iso': 200,
                'shutter': 1/250,
                'aperture': 5.6
            },
            'forest': {
                'iso': 400,
                'shutter': 1/125,
                'aperture': 4.0
            },
            'water': {
                'iso': 100,
                'shutter': 1/1000,
                'aperture': 5.6
            },
            'sunset': {
                'iso': 100,
                'shutter': 1/250,
                'aperture': 11.0
            }
        }
        
        return settings.get(scene_type, settings['landscape'])