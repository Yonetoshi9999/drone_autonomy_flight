"""
自動構図調整
"""

import cv2
import numpy as np

class AutoFraming:
    """自動構図調整クラス"""
    
    def __init__(self):
        self.rule_of_thirds_enabled = True
        self.golden_ratio_enabled = False
        
    def analyze_composition(self, frame):
        """構図解析"""
        
        height, width = frame.shape[:2]
        
        # 三分割法のガイドライン
        h_third = height // 3
        w_third = width // 3
        
        # 主要な特徴点を検出
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners = cv2.goodFeaturesToTrack(
            gray, 
            maxCorners=100, 
            qualityLevel=0.01, 
            minDistance=10
        )
        
        # 構図スコア計算
        score = self.calculate_composition_score(corners, width, height)
        
        return score
    
    def calculate_composition_score(self, features, width, height):
        """構図スコア計算"""
        
        if features is None:
            return 0
        
        score = 0
        h_third = height // 3
        w_third = width // 3
        
        # 三分割線近くの特徴点にボーナス
        for feature in features:
            x, y = feature[0]
            
            # 縦線との距離
            dist_v = min(abs(x - w_third), abs(x - 2*w_third))
            # 横線との距離
            dist_h = min(abs(y - h_third), abs(y - 2*h_third))
            
            if dist_v < 20:
                score += 10
            if dist_h < 20:
                score += 10
        
        return score
    
    def suggest_gimbal_adjustment(self, frame):
        """ジンバル調整提案"""
        
        score = self.analyze_composition(frame)
        
        suggestions = {
            'pitch_adjustment': 0,
            'yaw_adjustment': 0,
            'quality_score': score
        }
        
        # スコアが低い場合は調整を提案
        if score < 50:
            suggestions['pitch_adjustment'] = -5  # 5度下向き
            
        return suggestions