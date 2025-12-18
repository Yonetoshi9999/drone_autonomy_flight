"""
AirSim Sensor Interfaces

Provides interfaces to AirSim sensors including LiDAR, Camera, IMU, and GPS.
All sensors run at their specified update rates.
"""

import time
import logging
from typing import Optional, Tuple, Dict, Any
from dataclasses import dataclass

import numpy as np
import cv2

try:
    import airsim
except ImportError:
    airsim = None
    logging.warning("AirSim not installed. Install with: pip install airsim")


logger = logging.getLogger(__name__)


@dataclass
class LiDARData:
    """LiDAR scan data."""
    points: np.ndarray  # (N, 3) array of points in sensor frame
    ranges: np.ndarray  # (360,) array of ranges in meters
    angles: np.ndarray  # (360,) array of angles in radians
    timestamp: float


@dataclass
class CameraData:
    """Camera image data."""
    rgb: np.ndarray  # RGB image (H, W, 3)
    depth: Optional[np.ndarray]  # Depth image (H, W)
    segmentation: Optional[np.ndarray]  # Segmentation image (H, W)
    timestamp: float
    camera_info: Dict[str, Any]


@dataclass
class IMUData:
    """IMU sensor data at 400Hz."""
    linear_acceleration: np.ndarray  # (3,) m/s^2
    angular_velocity: np.ndarray  # (3,) rad/s
    orientation: np.ndarray  # (4,) quaternion [w, x, y, z]
    timestamp: float


@dataclass
class GPSData:
    """GPS sensor data at 10Hz."""
    latitude: float  # degrees
    longitude: float  # degrees
    altitude: float  # meters (MSL)
    velocity: np.ndarray  # (3,) m/s in NED
    fix_type: int  # 0=no fix, 3=3D fix, 4=RTK fix
    eph: float  # Horizontal position accuracy (m)
    epv: float  # Vertical position accuracy (m)
    timestamp: float


class AirSimSensors:
    """
    Interface to AirSim sensors for EDU650 drone.

    Provides access to:
    - RPLidar A2M8 (360° scanning, 10Hz)
    - Camera (1920x1080 @ 30fps)
    - IMU (400Hz update rate)
    - GPS (10Hz with RTK support)
    """

    def __init__(
        self,
        vehicle_name: str = "EDU650",
        airsim_host: str = "127.0.0.1",
        airsim_port: int = 41451,
    ):
        """
        Initialize AirSim sensors.

        Args:
            vehicle_name: Name of the vehicle in AirSim
            airsim_host: AirSim server host
            airsim_port: AirSim server port
        """
        if airsim is None:
            raise ImportError("AirSim is not installed")

        self.vehicle_name = vehicle_name
        self.airsim_host = airsim_host
        self.airsim_port = airsim_port

        # AirSim client
        self.client: Optional[airsim.MultirotorClient] = None
        self.connected = False

        # Sensor update times
        self.last_lidar_update = 0.0
        self.last_camera_update = 0.0
        self.last_imu_update = 0.0
        self.last_gps_update = 0.0

        # Sensor rates
        self.lidar_rate = 10.0  # Hz
        self.camera_rate = 30.0  # Hz
        self.imu_rate = 400.0  # Hz
        self.gps_rate = 10.0  # Hz

    def connect(self, timeout: float = 10.0) -> bool:
        """
        Connect to AirSim.

        Args:
            timeout: Connection timeout in seconds

        Returns:
            True if connected successfully, False otherwise
        """
        try:
            logger.info(f"Connecting to AirSim at {self.airsim_host}:{self.airsim_port}")

            self.client = airsim.MultirotorClient(
                ip=self.airsim_host,
                port=self.airsim_port,
            )

            start_time = time.time()
            while time.time() - start_time < timeout:
                try:
                    self.client.confirmConnection()
                    self.connected = True
                    logger.info("Connected to AirSim")
                    return True
                except:
                    time.sleep(0.5)

            logger.error("AirSim connection timeout")
            return False

        except Exception as e:
            logger.error(f"Failed to connect to AirSim: {e}")
            return False

    def disconnect(self):
        """Disconnect from AirSim."""
        self.connected = False
        logger.info("Disconnected from AirSim")

    def get_lidar_data(self) -> Optional[LiDARData]:
        """
        Get LiDAR data from RPLidar A2M8.

        Returns 360° scan at 10Hz update rate.

        Returns:
            LiDARData object or None if data not available
        """
        if not self.connected:
            return None

        current_time = time.time()

        # Check if we should update (10Hz rate)
        if current_time - self.last_lidar_update < 1.0 / self.lidar_rate:
            return None

        try:
            # Get LiDAR data from AirSim
            lidar_data = self.client.getLidarData(
                lidar_name="LidarFront",
                vehicle_name=self.vehicle_name,
            )

            if len(lidar_data.point_cloud) < 3:
                return None

            # Convert point cloud to numpy array
            points = np.array(lidar_data.point_cloud, dtype=np.float32)
            points = points.reshape(-1, 3)

            # Convert to 360-degree scan (simplification for 2D navigation)
            ranges, angles = self._points_to_scan(points, num_rays=360)

            self.last_lidar_update = current_time

            return LiDARData(
                points=points,
                ranges=ranges,
                angles=angles,
                timestamp=current_time,
            )

        except Exception as e:
            logger.error(f"Failed to get LiDAR data: {e}")
            return None

    def _points_to_scan(
        self,
        points: np.ndarray,
        num_rays: int = 360,
        max_range: float = 16.0,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Convert 3D point cloud to 2D scan.

        Args:
            points: (N, 3) array of 3D points
            num_rays: Number of rays in scan
            max_range: Maximum range in meters

        Returns:
            Tuple of (ranges, angles) arrays
        """
        # Project points to XY plane
        xy_points = points[:, :2]

        # Calculate angles and ranges
        angles_rad = np.arctan2(xy_points[:, 1], xy_points[:, 0])
        ranges = np.linalg.norm(xy_points, axis=1)

        # Create bins for 360-degree scan
        angle_bins = np.linspace(-np.pi, np.pi, num_rays + 1)
        scan_ranges = np.full(num_rays, max_range)
        scan_angles = np.linspace(-np.pi, np.pi, num_rays)

        # Bin points by angle and take minimum range in each bin
        for i in range(num_rays):
            mask = (angles_rad >= angle_bins[i]) & (angles_rad < angle_bins[i + 1])
            if np.any(mask):
                scan_ranges[i] = min(np.min(ranges[mask]), max_range)

        return scan_ranges, scan_angles

    def get_camera_data(
        self,
        get_depth: bool = True,
        get_segmentation: bool = False,
    ) -> Optional[CameraData]:
        """
        Get camera data at 30fps.

        Args:
            get_depth: Whether to get depth image
            get_segmentation: Whether to get segmentation image

        Returns:
            CameraData object or None if data not available
        """
        if not self.connected:
            return None

        current_time = time.time()

        # Check if we should update (30Hz rate)
        if current_time - self.last_camera_update < 1.0 / self.camera_rate:
            return None

        try:
            # Request image types
            requests = [
                airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False),
            ]

            if get_depth:
                requests.append(
                    airsim.ImageRequest("front_center", airsim.ImageType.DepthPlanar, True, False)
                )

            if get_segmentation:
                requests.append(
                    airsim.ImageRequest("front_center", airsim.ImageType.Segmentation, False, False)
                )

            # Get images
            responses = self.client.simGetImages(requests, vehicle_name=self.vehicle_name)

            # Process RGB image
            rgb_response = responses[0]
            rgb_array = np.frombuffer(rgb_response.image_data_uint8, dtype=np.uint8)
            rgb = rgb_array.reshape(rgb_response.height, rgb_response.width, 3)
            rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)

            # Process depth image if requested
            depth = None
            if get_depth and len(responses) > 1:
                depth_response = responses[1]
                depth = np.array(depth_response.image_data_float, dtype=np.float32)
                depth = depth.reshape(depth_response.height, depth_response.width)

            # Process segmentation image if requested
            segmentation = None
            if get_segmentation and len(responses) > (2 if get_depth else 1):
                seg_response = responses[2 if get_depth else 1]
                seg_array = np.frombuffer(seg_response.image_data_uint8, dtype=np.uint8)
                segmentation = seg_array.reshape(seg_response.height, seg_response.width, 3)

            self.last_camera_update = current_time

            return CameraData(
                rgb=rgb,
                depth=depth,
                segmentation=segmentation,
                timestamp=current_time,
                camera_info={
                    'width': rgb_response.width,
                    'height': rgb_response.height,
                    'fov': 90.0,  # From settings.json
                },
            )

        except Exception as e:
            logger.error(f"Failed to get camera data: {e}")
            return None

    def get_imu_data(self) -> Optional[IMUData]:
        """
        Get IMU data at 400Hz.

        Returns:
            IMUData object or None if data not available
        """
        if not self.connected:
            return None

        current_time = time.time()

        # Check if we should update (400Hz rate)
        if current_time - self.last_imu_update < 1.0 / self.imu_rate:
            return None

        try:
            # Get IMU data from AirSim
            imu_data = self.client.getImuData(
                imu_name="Imu",
                vehicle_name=self.vehicle_name,
            )

            # Extract data
            linear_acc = np.array([
                imu_data.linear_acceleration.x_val,
                imu_data.linear_acceleration.y_val,
                imu_data.linear_acceleration.z_val,
            ])

            angular_vel = np.array([
                imu_data.angular_velocity.x_val,
                imu_data.angular_velocity.y_val,
                imu_data.angular_velocity.z_val,
            ])

            orientation = np.array([
                imu_data.orientation.w_val,
                imu_data.orientation.x_val,
                imu_data.orientation.y_val,
                imu_data.orientation.z_val,
            ])

            self.last_imu_update = current_time

            return IMUData(
                linear_acceleration=linear_acc,
                angular_velocity=angular_vel,
                orientation=orientation,
                timestamp=current_time,
            )

        except Exception as e:
            logger.error(f"Failed to get IMU data: {e}")
            return None

    def get_gps_data(self) -> Optional[GPSData]:
        """
        Get GPS data at 10Hz with RTK support.

        Returns:
            GPSData object or None if data not available
        """
        if not self.connected:
            return None

        current_time = time.time()

        # Check if we should update (10Hz rate)
        if current_time - self.last_gps_update < 1.0 / self.gps_rate:
            return None

        try:
            # Get GPS data from AirSim
            gps_data = self.client.getGpsData(
                gps_name="Gps",
                vehicle_name=self.vehicle_name,
            )

            # Get velocity from kinematics
            kinematics = self.client.simGetGroundTruthKinematics(
                vehicle_name=self.vehicle_name,
            )

            velocity = np.array([
                kinematics.linear_velocity.x_val,
                kinematics.linear_velocity.y_val,
                kinematics.linear_velocity.z_val,
            ])

            self.last_gps_update = current_time

            return GPSData(
                latitude=gps_data.gnss.geo_point.latitude,
                longitude=gps_data.gnss.geo_point.longitude,
                altitude=gps_data.gnss.geo_point.altitude,
                velocity=velocity,
                fix_type=3,  # Assume 3D fix in simulation
                eph=gps_data.gnss.eph / 100.0,  # Convert cm to m
                epv=gps_data.gnss.epv / 100.0,
                timestamp=current_time,
            )

        except Exception as e:
            logger.error(f"Failed to get GPS data: {e}")
            return None

    def reset(self):
        """Reset all sensor timestamps."""
        self.last_lidar_update = 0.0
        self.last_camera_update = 0.0
        self.last_imu_update = 0.0
        self.last_gps_update = 0.0
