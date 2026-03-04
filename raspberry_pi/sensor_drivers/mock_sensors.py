"""
モックセンサー（テスト用）
"""

import numpy as np

class MockSensors:
    """テスト用モックセンサー"""
    
    @staticmethod
    def get_mock_lidar_data():
        """モックLiDARデータ"""
        scan = []
        for angle in range(0, 360, 5):
            quality = np.random.randint(10, 15)
            distance = np.random.uniform(500, 5000)
            scan.append((quality, angle, distance))
        return scan
    
    @staticmethod
    def get_mock_camera_frame():
        """モックカメラフレーム"""
        return np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)