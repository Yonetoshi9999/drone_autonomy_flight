"""
構図分析
"""

import cv2
import numpy as np

class CompositionAnalyzer:
    """構図分析クラス"""
    
    def __init__(self):
        self.golden_ratio = 1.618
        
    def analyze(self, frame):
        """構図分析"""
        
        analysis = {
            'rule_of_thirds': self.check_rule_of_thirds(frame),
            'golden_ratio': self.check_golden_ratio(frame),
            'leading_lines': self.detect_leading_lines(frame),
            'symmetry': self.check_symmetry(frame),
            'balance': self.check_balance(frame)
        }
        
        # 総合スコア
        analysis['total_score'] = self.calculate_total_score(analysis)
        
        return analysis
    
    def check_rule_of_thirds(self, frame):
        """三分割法チェック"""
        
        height, width = frame.shape[:2]
        
        # 三分割線
        h_lines = [height // 3, 2 * height // 3]
        v_lines = [width // 3, 2 * width // 3]
        
        # 特徴点検出
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=50,
            qualityLevel=0.01,
            minDistance=10
        )
        
        if corners is None:
            return 0
        
        score = 0
        
        # 交点付近の特徴点を評価
        for corner in corners:
            x, y = corner[0]
            
            for h_line in h_lines:
                for v_line in v_lines:
                    dist = np.sqrt((x - v_line)**2 + (y - h_line)**2)
                    if dist < 30:  # 30ピクセル以内
                        score += 10
        
        return min(100, score)
    
    def check_golden_ratio(self, frame):
        """黄金比チェック"""
        
        height, width = frame.shape[:2]
        
        # 黄金比分割点
        golden_h = int(height / self.golden_ratio)
        golden_w = int(width / self.golden_ratio)
        
        # エッジ検出
        edges = cv2.Canny(frame, 50, 150)
        
        # 黄金比線上のエッジ密度
        h_density = np.mean(edges[golden_h, :])
        v_density = np.mean(edges[:, golden_w])
        
        score = (h_density + v_density) / 2
        
        return min(100, score)
    
    def detect_leading_lines(self, frame):
        """リーディングライン検出"""
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        
        # Hough変換で直線検出
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi/180,
            threshold=100,
            minLineLength=100,
            maxLineGap=10
        )
        
        if lines is None:
            return 0
        
        # 対角線方向の線を評価
        diagonal_lines = 0
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = np.abs(np.arctan2(y2 - y1, x2 - x1))
            
            # 15-75度の線を対角線とみなす
            if np.pi/12 < angle < 5*np.pi/12:
                diagonal_lines += 1
        
        score = min(100, diagonal_lines * 10)
        
        return score
    
    def check_symmetry(self, frame):
        """対称性チェック"""
        
        height, width = frame.shape[:2]
        
        # 左右対称性
        left_half = frame[:, :width//2]
        right_half = cv2.flip(frame[:, width//2:], 1)
        
        # サイズ調整
        min_width = min(left_half.shape[1], right_half.shape[1])
        left_half = left_half[:, :min_width]
        right_half = right_half[:, :min_width]
        
        # 類似度計算
        diff = cv2.absdiff(left_half, right_half)
        similarity = 1.0 - (np.mean(diff) / 255.0)
        
        return similarity * 100
    
    def check_balance(self, frame):
        """バランスチェック"""
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        height, width = gray.shape
        
        # 重心計算
        y_coords, x_coords = np.mgrid[0:height, 0:width]
        
        total_intensity = np.sum(gray)
        if total_intensity == 0:
            return 50
        
        center_x = np.sum(x_coords * gray) / total_intensity
        center_y = np.sum(y_coords * gray) / total_intensity
        
        # 中心からのずれ
        ideal_center_x = width / 2
        ideal_center_y = height / 2
        
        deviation_x = abs(center_x - ideal_center_x) / ideal_center_x
        deviation_y = abs(center_y - ideal_center_y) / ideal_center_y
        
        # スコア計算（ずれが少ないほど高得点）
        score = 100 * (1 - (deviation_x + deviation_y) / 2)
        
        return max(0, score)
    
    def calculate_total_score(self, analysis):
        """総合スコア計算"""
        
        weights = {
            'rule_of_thirds': 0.3,
            'golden_ratio': 0.2,
            'leading_lines': 0.2,
            'symmetry': 0.15,
            'balance': 0.15
        }
        
        total = 0
        for key, weight in weights.items():
            if key in analysis:
                total += analysis[key] * weight
        
        return total