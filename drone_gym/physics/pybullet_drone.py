"""PyBullet-based drone physics simulation."""

import numpy as np
import pybullet as p
import pybullet_data
import os
from typing import Tuple, Optional, Dict, List


class PyBulletDrone:
    """
    PyBullet physics simulation for quadcopter drone.

    Simulates drone dynamics, sensors, and collision detection.
    Replaces AirSim for lightweight RL training.
    """

    def __init__(
        self,
        gui: bool = False,
        drone_model: str = "medium_quad",  # Default: 2.0kg medium quadcopter (matches ArduPilot)
        gravity: float = -9.81,
        timestep: float = 0.01,
        frame_skip: int = 1,
    ):
        """
        Initialize PyBullet drone simulation.

        Args:
            gui: Whether to show PyBullet GUI
            drone_model: Drone model type ("medium_quad" or "cf2x")
            gravity: Gravity acceleration (m/s^2)
            timestep: Simulation timestep (seconds)
            frame_skip: Number of physics steps per control step
        """
        self.gui = gui
        self.drone_model = drone_model
        self.gravity = gravity
        self.timestep = timestep
        self.frame_skip = frame_skip

        # Physics client
        self.client_id = None
        self.drone_id = None

        # Drone state
        self.position = np.zeros(3)
        self.orientation = np.zeros(4)  # quaternion [x, y, z, w]
        self.linear_velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)

        # Sensor configurations
        self.lidar_rays = 360
        self.lidar_range = 10.0  # meters
        self.camera_width = 64
        self.camera_height = 64

        # Obstacles
        self.obstacle_ids = []

        # Set physics parameters based on model
        self._set_model_parameters()

        # Current motor RPMs
        self.motor_rpms = np.zeros(4)

    def _set_model_parameters(self):
        """Set physics parameters based on drone model."""
        if self.drone_model == "medium_quad":
            # Medium quadcopter — parameters synchronized with sysid_params.txt
            # MASS=2.0  ARM_LENGTH=0.225  MOTOR_KV=920  MAX_THRUST=8.0
            # IXX=0.0347  IYY=0.0458  IZZ=0.0977  MOMENT_COEFF=0.016
            self.mass = 2.0           # kg  (MASS)
            self.arm_length = 0.225   # m   (ARM_LENGTH)
            self.thrust_to_weight = 1.6315  # = MAX_THRUST*4 / (MASS*g) = 32/19.613
            self.kf = 1.0e-5          # N/(rad/s)^2, thrust coefficient
            self.km = 1.6e-7          # N·m/(rad/s)^2 = MOMENT_COEFF * kf = 0.016 * 1e-5
            self.max_rpm = 894        # rad/s at MAX_THRUST = sqrt(8.0/1e-5)
            self.gnd_eff_coeff = 11.36859
            self.motor_kv = 920.0     # RPM/V  (MOTOR_KV)
            self.max_thrust_per_motor = 8.0   # N  (MAX_THRUST)

            # Motor positions for X configuration (larger frame)
            # Motor layout:   1(CCW)  0(CW)
            #                    \ /
            #                     X
            #                    / \
            #                2(CW)  3(CCW)
            self.motor_positions = np.array([
                [0.159, -0.159, 0.0],   # Motor 0 (front-right, CW)
                [-0.159, -0.159, 0.0],  # Motor 1 (rear-right, CCW)
                [-0.159, 0.159, 0.0],   # Motor 2 (rear-left, CW)
                [0.159, 0.159, 0.0],    # Motor 3 (front-left, CCW)
            ])

        elif self.drone_model == "cf2x":
            # Crazyflie 2.x - 0.027 kg nano-quadcopter
            self.mass = 0.027  # kg
            self.arm_length = 0.0397  # m
            self.thrust_to_weight = 2.25
            self.kf = 3.16e-10  # thrust coefficient
            self.km = 7.94e-12  # torque coefficient
            self.max_rpm = 21702  # RPM at max thrust
            self.gnd_eff_coeff = 11.36859

            # Motor positions for X configuration (small frame)
            self.motor_positions = np.array([
                [0.028, -0.028, 0.0],   # Motor 0 (front-right, CW)
                [-0.028, -0.028, 0.0],  # Motor 1 (rear-right, CCW)
                [-0.028, 0.028, 0.0],   # Motor 2 (rear-left, CW)
                [0.028, 0.028, 0.0],    # Motor 3 (front-left, CCW)
            ])

        else:
            raise ValueError(
                f"Unknown drone model: {self.drone_model}. "
                f"Available models: 'medium_quad', 'cf2x'"
            )

    def connect(self):
        """Connect to PyBullet physics server."""
        if self.gui:
            self.client_id = p.connect(p.GUI)
        else:
            self.client_id = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, self.gravity)
        p.setTimeStep(self.timestep)

        # Load ground plane
        p.loadURDF("plane.urdf")

        # Create simple drone model (sphere with arms)
        self._create_drone()

    def _create_drone(self):
        """Load quadcopter model from URDF file."""
        # Get the path to the URDF file
        current_dir = os.path.dirname(os.path.abspath(__file__))
        assets_dir = os.path.join(os.path.dirname(current_dir), "assets")
        urdf_path = os.path.join(assets_dir, f"{self.drone_model}.urdf")

        if not os.path.exists(urdf_path):
            raise FileNotFoundError(
                f"URDF file not found: {urdf_path}\n"
                f"Please ensure the drone model '{self.drone_model}' exists in {assets_dir}"
            )

        # Load URDF
        self.drone_id = p.loadURDF(
            urdf_path,
            basePosition=[0, 0, 1.0],
            baseOrientation=[0, 0, 0, 1],
            flags=p.URDF_USE_INERTIA_FROM_FILE,
        )

        # Set initial position
        self.reset_pose([0, 0, 1.0], [0, 0, 0, 1])

    def reset_pose(self, position: List[float], orientation: List[float]):
        """
        Reset drone pose.

        Args:
            position: [x, y, z] in meters
            orientation: [x, y, z, w] quaternion
        """
        p.resetBasePositionAndOrientation(
            self.drone_id, position, orientation
        )
        p.resetBaseVelocity(
            self.drone_id,
            linearVelocity=[0, 0, 0],
            angularVelocity=[0, 0, 0],
        )
        self.update_state()

    def disconnect(self):
        """Disconnect from PyBullet."""
        if self.client_id is not None:
            p.disconnect(self.client_id)
            self.client_id = None

    def _velocity_to_rpms(self, action: np.ndarray) -> np.ndarray:
        """
        Convert velocity commands to motor RPMs using a simple controller.

        Args:
            action: [vx, vy, vz, yaw_rate] velocity commands

        Returns:
            motor_rpms: RPM for each of the 4 motors
        """
        # Extract commands
        target_vel = action[:3]
        yaw_rate = action[3] if len(action) > 3 else 0.0

        current_vel = self.linear_velocity

        # PD controller gains (scaled based on mass)
        if self.drone_model == "medium_quad":
            # Gains for 2.0kg medium quadcopter
            kp_z = 20.0 * self.mass
            kd_z = 5.0 * self.mass
            kp_xy = 8.0 * self.mass
            kd_xy = 2.0 * self.mass
            k_yaw = 1.0 * self.mass
        else:
            # Gains for Crazyflie (original values)
            kp_z = 5000.0
            kd_z = 500.0
            kp_xy = 1000.0
            kd_xy = 200.0
            k_yaw = 500.0

        # Compute thrust (for altitude control)
        # Velocity tracking controller: P control on velocity error
        z_error = target_vel[2] - current_vel[2]

        # NOTE: Removed problematic D term that was damping absolute velocity
        # Old (broken): thrust = hover_thrust + kp_z * z_error - kd_z * current_vel[2]
        # Problem: D term fought upward motion, causing drone to descend
        # New: Pure P controller on velocity error (stable for velocity tracking)
        thrust = self.mass * abs(self.gravity) + kp_z * z_error

        # Compute attitude commands from horizontal velocities
        # For X configuration quadcopter
        vel_error_x = target_vel[0] - current_vel[0]
        vel_error_y = target_vel[1] - current_vel[1]

        # Convert to roll/pitch commands (simplified)
        # Positive vx -> negative pitch (lean forward)
        # Positive vy -> positive roll (lean right)
        pitch_cmd = -kp_xy * vel_error_x - kd_xy * current_vel[0]
        roll_cmd = kp_xy * vel_error_y + kd_xy * current_vel[1]

        # Yaw torque
        yaw_cmd = k_yaw * yaw_rate

        # Motor mixing for X configuration
        # thrust: sum of all motors
        # roll: difference between left and right motors
        # pitch: difference between front and back motors
        # yaw: alternating signs (CW vs CCW motors)
        base_thrust = thrust / 4.0

        motor_thrusts = np.array([
            base_thrust + pitch_cmd - roll_cmd - yaw_cmd,  # Motor 0 (FR, CW)
            base_thrust - pitch_cmd - roll_cmd + yaw_cmd,  # Motor 1 (RR, CCW)
            base_thrust - pitch_cmd + roll_cmd - yaw_cmd,  # Motor 2 (RL, CW)
            base_thrust + pitch_cmd + roll_cmd + yaw_cmd,  # Motor 3 (FL, CCW)
        ])

        # Clip thrusts to valid range
        max_thrust = self.mass * abs(self.gravity) * self.thrust_to_weight / 4.0
        motor_thrusts = np.clip(motor_thrusts, 0, max_thrust)

        # Convert thrust to RPM: thrust = kf * rpm^2
        motor_rpms = np.sqrt(motor_thrusts / self.kf)

        return motor_rpms

    def _apply_motor_forces(self, motor_rpms: np.ndarray):
        """
        Apply forces and torques from motor RPMs.

        Args:
            motor_rpms: RPM for each of the 4 motors
        """
        # Calculate thrust from each motor: F = kf * rpm^2
        motor_thrusts = self.kf * (motor_rpms ** 2)

        # Calculate torque from each motor: T = km * rpm^2
        # CW motors (0, 2) produce negative yaw torque
        # CCW motors (1, 3) produce positive yaw torque
        motor_torques = self.km * (motor_rpms ** 2) * np.array([-1, 1, -1, 1])

        # Total thrust (in body z direction)
        total_thrust = np.sum(motor_thrusts)

        # Apply thrust force in world frame (upward)
        rot_matrix = np.array(p.getMatrixFromQuaternion(self.orientation)).reshape(3, 3)
        thrust_world = rot_matrix @ np.array([0, 0, total_thrust])

        p.applyExternalForce(
            self.drone_id,
            -1,  # base link
            thrust_world,
            [0, 0, 0],
            p.WORLD_FRAME,
        )

        # Compute body torques from thrust differences
        # Roll torque (around x-axis)
        roll_torque = self.arm_length * (motor_thrusts[3] + motor_thrusts[2]
                                         - motor_thrusts[0] - motor_thrusts[1])

        # Pitch torque (around y-axis)
        pitch_torque = self.arm_length * (motor_thrusts[0] + motor_thrusts[3]
                                          - motor_thrusts[1] - motor_thrusts[2])

        # Yaw torque (around z-axis)
        yaw_torque = np.sum(motor_torques)

        # Apply torques in body frame
        torques = np.array([roll_torque, pitch_torque, yaw_torque])
        p.applyExternalTorque(
            self.drone_id,
            -1,
            torques,
            p.LINK_FRAME,
        )

    def step(self, action: np.ndarray) -> bool:
        """
        Apply control action and step physics.

        Args:
            action: Control input [vx, vy, vz, yaw_rate]

        Returns:
            True if collision detected
        """
        # Convert velocity command to motor RPMs
        self.motor_rpms = self._velocity_to_rpms(action)

        # Step simulation
        for _ in range(self.frame_skip):
            # Apply motor forces/torques
            self._apply_motor_forces(self.motor_rpms)

            # Step physics
            p.stepSimulation()

        # Update state
        self.update_state()

        # Check collision
        collision = self.check_collision()

        return collision

    def update_state(self):
        """Update drone state from physics engine."""
        pos, orn = p.getBasePositionAndOrientation(self.drone_id)
        lin_vel, ang_vel = p.getBaseVelocity(self.drone_id)

        self.position = np.array(pos)
        self.orientation = np.array(orn)
        self.linear_velocity = np.array(lin_vel)
        self.angular_velocity = np.array(ang_vel)

    def check_collision(self) -> bool:
        """
        Check if drone collided with obstacles.

        Returns:
            True if collision detected
        """
        contact_points = p.getContactPoints(bodyA=self.drone_id)
        return len(contact_points) > 0

    def get_state(self) -> Dict[str, np.ndarray]:
        """
        Get current drone state.

        Returns:
            Dictionary with position, orientation, velocities
        """
        # Convert quaternion to Euler angles
        euler = p.getEulerFromQuaternion(self.orientation)

        return {
            "position": self.position.copy(),
            "orientation_quat": self.orientation.copy(),
            "orientation_euler": np.array(euler),
            "linear_velocity": self.linear_velocity.copy(),
            "angular_velocity": self.angular_velocity.copy(),
        }

    def get_lidar_scan(self) -> np.ndarray:
        """
        Simulate 360-degree 2D LiDAR scan.

        Returns:
            Array of distances [360] in meters
        """
        distances = np.ones(self.lidar_rays) * self.lidar_range

        pos = self.position

        for i in range(self.lidar_rays):
            angle = 2 * np.pi * i / self.lidar_rays

            # Ray direction in world frame
            ray_dir = np.array([
                np.cos(angle),
                np.sin(angle),
                0.0,
            ])

            ray_from = pos
            ray_to = pos + ray_dir * self.lidar_range

            result = p.rayTest(ray_from, ray_to)

            if result and result[0][0] != -1:  # Hit something
                hit_fraction = result[0][2]
                distances[i] = hit_fraction * self.lidar_range

        return distances

    def get_camera_image(self) -> np.ndarray:
        """
        Get camera RGB image from drone perspective.

        Returns:
            RGB image array [height, width, 3]
        """
        # Camera parameters
        pos = self.position
        orn = self.orientation

        # Compute view matrix (camera looks forward)
        rot_matrix = p.getMatrixFromQuaternion(orn)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)

        # Forward direction
        forward = rot_matrix @ np.array([1, 0, 0])
        up = rot_matrix @ np.array([0, 0, 1])

        target = pos + forward * 5.0

        view_matrix = p.computeViewMatrix(
            cameraEyePosition=pos.tolist(),
            cameraTargetPosition=target.tolist(),
            cameraUpVector=up.tolist(),
        )

        proj_matrix = p.computeProjectionMatrixFOV(
            fov=60.0,
            aspect=1.0,
            nearVal=0.1,
            farVal=100.0,
        )

        # Render image
        width, height = self.camera_width, self.camera_height
        img = p.getCameraImage(
            width,
            height,
            view_matrix,
            proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL if self.gui else p.ER_TINY_RENDERER,
        )

        # Extract RGB - img[2] is the pixel data
        # Convert to numpy array with proper shape
        rgb_array = np.array(img[2], dtype=np.uint8).reshape(height, width, 4)

        # Remove alpha channel and return RGB only
        rgb = rgb_array[:, :, :3].copy()

        return rgb

    def add_obstacle(
        self,
        position: List[float],
        size: List[float],
        obstacle_type: str = "box",
    ) -> int:
        """
        Add obstacle to environment.

        Args:
            position: [x, y, z] position
            size: Size parameters (depends on type)
            obstacle_type: "box", "sphere", or "cylinder"

        Returns:
            Obstacle ID
        """
        if obstacle_type == "box":
            collision_shape = p.createCollisionShape(
                p.GEOM_BOX, halfExtents=size
            )
            visual_shape = p.createVisualShape(
                p.GEOM_BOX, halfExtents=size, rgbaColor=[0.8, 0.1, 0.1, 1.0]
            )
        elif obstacle_type == "sphere":
            radius = size[0]
            collision_shape = p.createCollisionShape(
                p.GEOM_SPHERE, radius=radius
            )
            visual_shape = p.createVisualShape(
                p.GEOM_SPHERE, radius=radius, rgbaColor=[0.8, 0.1, 0.1, 1.0]
            )
        elif obstacle_type == "cylinder":
            radius, length = size[0], size[1]
            collision_shape = p.createCollisionShape(
                p.GEOM_CYLINDER, radius=radius, height=length
            )
            visual_shape = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=radius,
                length=length,
                rgbaColor=[0.8, 0.1, 0.1, 1.0],
            )
        else:
            raise ValueError(f"Unknown obstacle type: {obstacle_type}")

        obstacle_id = p.createMultiBody(
            baseMass=0,  # Static obstacle
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=position,
        )

        self.obstacle_ids.append(obstacle_id)
        return obstacle_id

    def clear_obstacles(self):
        """Remove all obstacles from environment."""
        for obs_id in self.obstacle_ids:
            p.removeBody(obs_id)
        self.obstacle_ids.clear()

    def set_camera_view(self, distance: float = 5.0, yaw: float = 0, pitch: float = -30):
        """
        Set debug camera view (GUI mode only).

        Args:
            distance: Distance from target
            yaw: Yaw angle (degrees)
            pitch: Pitch angle (degrees)
        """
        if self.gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=distance,
                cameraYaw=yaw,
                cameraPitch=pitch,
                cameraTargetPosition=self.position,
            )
