#!/usr/bin/env python3
"""
ArduPilot Mode 99 Gym Environment for Reinforcement Learning

This environment wraps ArduPilot SITL with Mode 99 (LQR controller)
to enable reinforcement learning for high-level mission planning.

Architecture:
    RL Agent → Position/Velocity/Yaw Targets → Mode 99 LQR → Motors
"""

import gymnasium as gym
from gymnasium import spaces
import numpy as np
from pymavlink import mavutil
import time
import os
import subprocess
from typing import Tuple, Dict, Any, Optional


class ArduPilotMode99Env(gym.Env):
    """
    Gym environment for training RL agents with ArduPilot Mode 99

    State Space (25D):
        - Position: [x, y, z] (NED frame, meters)
        - Velocity: [vx, vy, vz] (NED frame, m/s)
        - Attitude: [roll, pitch, yaw] (radians)
        - Rates: [p, q, r] (rad/s)
        - Battery: voltage
        - GPS: [lat, lon, alt, num_sats]
        - Obstacle distances: [front, back, left, right, up, down] (meters)
        - Goal relative position: [dx, dy, dz]

    Action Space (4D):
        - vel_toward_goal: speed toward goal [0, MAX_VEL] m/s (always positive → random policy drives toward goal)
        - vel_lateral: lateral speed [-MAX_VEL, MAX_VEL] m/s (perpendicular to goal direction, for obstacle avoidance)
        - vel_D: vertical speed [-MAX_VEL_D, MAX_VEL_D] m/s (NED: positive = descend)
        - yaw_rate: yaw rate (rad/s)

    Reward:
        - Goal reaching: +100
        - Distance to goal: -0.1 * distance
        - Obstacle penalty: -10 * exp(-dist) for dist < 5m
        - Crash penalty: -1000
        - Energy penalty: -0.01 * ||velocity||
        - Smoothness penalty: -0.05 * ||action_delta||

    Obstacle simulation:
        Virtual box obstacles are placed in NED space each episode.
        6-direction LiDAR distances are computed geometrically from drone position.
        This replicates the RPi's physical LiDAR output at runtime.
    """

    metadata = {'render.modes': ['human']}

    # 6-direction LiDAR rays in NED frame: front(+N), back(-N), right(+E), left(-E), up(-D), down(+D)
    _LIDAR_DIRS = np.array([
        [1, 0, 0],   # front (+N)
        [-1, 0, 0],  # back  (-N)
        [0, 1, 0],   # right (+E)
        [0, -1, 0],  # left  (-E)
        [0, 0, -1],  # up    (-D)
        [0, 0, 1],   # down  (+D)
    ], dtype=np.float32)
    _LIDAR_RANGE = 10.0  # meters, max sensing range

    def __init__(
        self,
        sitl_connection: str = 'tcp:127.0.0.1:5760',
        mission_type: str = 'obstacle_avoidance',
        max_steps: int = 1500,
        goal_radius: float = 10.0,
        time_scale: float = 1.0,
        enable_obstacles: bool = True,
        goal_dist_min: float = 5.0,
        goal_dist_max: float = 15.0
    ):
        """
        Initialize ArduPilot Mode 99 Gym Environment

        Args:
            sitl_connection: MAVLink connection string
            mission_type: 'obstacle_avoidance' or 'waypoint_navigation'
            max_steps: Maximum steps per episode
            goal_radius: Distance threshold for goal reaching (meters)
            time_scale: Simulation speed multiplier
            enable_obstacles: Whether to include obstacles
            goal_dist_min: Minimum goal distance (meters)
            goal_dist_max: Maximum goal distance (meters)
        """
        super().__init__()

        self.sitl_connection = sitl_connection
        self.mission_type = mission_type
        self.max_steps = max_steps
        self.goal_radius = goal_radius
        self.time_scale = time_scale
        self.enable_obstacles = enable_obstacles
        self.goal_dist_min = goal_dist_min
        self.goal_dist_max = goal_dist_max

        # State space: 23D continuous
        # vel(3) + att(3) + rates(3) + battery(1) + gps(4) + obstacles(6) + goal_rel(3)
        # NOTE: absolute position removed — it accumulates unboundedly across episodes
        # and confuses the policy. goal_rel(3) is sufficient for direction.
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(23,),
            dtype=np.float32
        )

        # Action space: 4D continuous — goal-relative coordinate frame
        # action[0] = vel_toward_goal: [MIN_VEL_FWD, MAX_VEL]
        #             Lower bound = 1.5 m/s: drone ALWAYS moves toward goal (no hovering).
        #             SB3 PPO initializes with NN output≈0 → maps to midpoint ≈ 2.75 m/s.
        #             Guarantees goal-directed flight from step 1.
        # action[1] = vel_lateral: [-MAX_VEL_LAT, MAX_VEL_LAT] — perpendicular to goal dir
        #             Reduced to ±1.0 m/s to prevent lateral drift from dominating
        # action[2] = vel_D: [-MAX_VEL_D, MAX_VEL_D] — vertical (NED: + = descend)
        # action[3] = yaw_rate: [-0.3, 0.3] rad/s
        # Converted to NED frame in step() before sending to Mode 99
        MAX_VEL = 3.5       # m/s horizontal max (tilt_scale handles recovery; was 2.5)
        MIN_VEL_FWD = 1.5   # m/s min toward-goal speed when outside goal (prevents hovering far away)
        MAX_VEL_LAT = 1.0   # m/s lateral max (reduced to prevent lateral drift)
        MAX_VEL_D = 0.3     # m/s vertical (conservative)
        self.min_vel_fwd = MIN_VEL_FWD
        self.action_space = spaces.Box(
            low=np.array([0.0,      -MAX_VEL_LAT, -MAX_VEL_D, -0.3]),
            high=np.array([MAX_VEL,  MAX_VEL_LAT,  MAX_VEL_D,  0.3]),
            shape=(4,),
            dtype=np.float32
        )

        # MAVLink connection
        self.mav = None
        self._ekf_ready = False  # set True after connect() EKF token passes
        self.connect()
        self._ekf_ready = True

        # Episode state
        self.current_step = 0
        self.prev_action = np.zeros(4)
        self.prev_goal_dist = None
        self.initial_goal_dist = 0.0
        self._approached_goal = False  # True once drone enters goal_radius zone
        self._goal_reached_flag = False  # True once drone first enters goal_radius (one-time)
        self.goal_position = np.zeros(3)
        self.start_position = np.zeros(3)
        self.episode_reward = 0.0
        self._target_pos = np.zeros(3)  # integrated position target (N, E, D)

        # Telemetry data
        self.telemetry = {
            'position': np.zeros(3),
            'velocity': np.zeros(3),
            'attitude': np.zeros(3),
            'rates': np.zeros(3),
            'battery': 0.0,
            'gps': np.zeros(4),
            'obstacles': np.full(6, 10.0),  # Initialize with safe distances
            'armed': False,
            'mode': 0,
            'lqi_diag': {}  # LQI diagnostics from NAMED_VALUE_FLOAT (LQI_Thrust, EKF_pD, etc.)
        }

        # Mode 99 initial reference position (set during reset)
        self._mode99_ref = np.zeros(3)

        # Virtual obstacles for this episode: list of (center, half_size) in NED (meters)
        self._obstacles: list = []

    def _restart_sitl(self):
        """Kill and restart SITL process to reset position drift."""
        pid_file = '/tmp/sitl_mode99.pid'
        sitl_cmd_file = '/tmp/sitl_mode99_cmd.sh'

        # Kill existing arducopter (pkill covers orphan processes too)
        if os.path.exists(pid_file):
            with open(pid_file) as f:
                pid = f.read().strip()
            try:
                subprocess.run(['kill', pid], check=False)
            except Exception:
                pass
        subprocess.run(['pkill', '-f', 'arducopter'], check=False)
        # Fixed wait — do NOT use socket check (SITL TCP accepts only 1 connection)
        time.sleep(8.0)

        # Restart SITL — cmd file has "cd $ARDUPILOT_DIR &&" so physical model loads correctly
        if not os.path.exists(sitl_cmd_file):
            print(f"  ⚠️  SITL cmd file not found: {sitl_cmd_file} — skipping restart")
            return
        sitl_proc = subprocess.Popen(['bash', sitl_cmd_file],
                                     stdout=open('/tmp/sitl_mode99.log', 'w'),
                                     stderr=subprocess.STDOUT)
        with open(pid_file, 'w') as f:
            f.write(str(sitl_proc.pid))
        print(f"  ✅ SITL restarted (PID: {sitl_proc.pid})")

        # Clear EKF ready flag — connect() will re-wait for the EKF token
        self._ekf_ready = False

        # Reconnect MAVLink with retry
        if self.mav:
            try:
                self.mav.close()
            except Exception:
                pass
        for attempt in range(15):
            try:
                self.connect()
                print(f"  ✅ MAVLink reconnected after SITL restart")
                break
            except Exception as e:
                print(f"  ⚠️  MAVLink connect attempt {attempt+1}/15 failed: {e}")
                time.sleep(3.0)
        else:
            raise RuntimeError("Failed to reconnect MAVLink after SITL restart")

        # Wait for fresh LOCAL_POSITION_NED from restarted SITL and update telemetry.
        # Retry stream requests every 3s because SITL may not process them immediately.
        deadline = time.time() + 30.0
        last_request = 0.0
        while time.time() < deadline:
            # Re-request streams periodically until messages arrive
            if time.time() - last_request > 3.0:
                self.request_data_streams()
                last_request = time.time()
            msg = self.mav.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1.0)
            if msg:
                self.telemetry['position'] = np.array([msg.x, msg.y, msg.z])
                self.telemetry['velocity'] = np.array([msg.vx, msg.vy, msg.vz])
                print(f"  ✅ Telemetry refreshed: pos=({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f})")
                break
        else:
            print("  ⚠️  Telemetry refresh timeout — position may be stale")

    def connect(self):
        """Connect to ArduPilot SITL via MAVLink"""
        print(f"Connecting to ArduPilot SITL: {self.sitl_connection}")
        self.mav = mavutil.mavlink_connection(
            self.sitl_connection,
            source_system=255
        )

        # Wait for heartbeat
        print("Waiting for heartbeat...")
        self.mav.wait_heartbeat()
        print(f"✅ Connected to system {self.mav.target_system}, component {self.mav.target_component}")

        # Request data streams
        self.request_data_streams()

        # Wait for EKF/GPS ready — definitive token that SITL (--wipe) is ready to arm.
        # Primary: EKF_STATUS_REPORT pos+vel flags (from EXTRA3 stream).
        # Fallback: GPS fix >= 3 sustained for 3 consecutive messages if EKF msg absent.
        print("Waiting for EKF/GPS ready (SITL initialisation token)...")
        gps_ok = False
        ekf_ok = False
        while not (gps_ok and ekf_ok):
            msg = self.mav.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            mt = msg.get_type()
            if mt == 'GPS_RAW_INT':
                if msg.fix_type >= 3 and msg.satellites_visible >= 6:
                    gps_ok = True
            elif mt == 'EKF_STATUS_REPORT':
                has_vel = bool(msg.flags & 0x002)
                has_pos = bool(msg.flags & 0x010)
                if gps_ok and has_vel and has_pos:
                    ekf_ok = True
        print("✅ EKF/GPS ready — SITL initialisation complete")

    def request_data_streams(self):
        """Request telemetry data streams at specified rates"""
        # Position data (LOCAL_POSITION_NED) at 50Hz
        # SITL speedup=5 → 50Hz sim = 10Hz real, matching gym step rate
        self.mav.mav.request_data_stream_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            50,
            1
        )
        # Extended status (GPS_RAW_INT, SYS_STATUS) at 5Hz
        self.mav.mav.request_data_stream_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            5,
            1
        )
        # Attitude (ATTITUDE) at 50Hz
        self.mav.mav.request_data_stream_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            50,
            1
        )
        # EKF_STATUS_REPORT at 5Hz (in EXTRA3 stream)
        self.mav.mav.request_data_stream_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA3,
            5,
            1
        )

    def reset(
        self,
        seed: Optional[int] = None,
        options: Optional[Dict[str, Any]] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Reset environment to initial state.

        Startup sequence mirrors companion_mode99.py:
          1. Disable ARMING_CHECK
          2. Set GUIDED mode (while disarmed — lenient EKF check)
          3. Wait for GPS fix + EKF convergence
          4. Arm (force, magic 2989)
          5. Takeoff to 5m in GUIDED
          6. Switch to Mode 99
          7. Capture M99_REF_* initial reference from flight controller
          8. Begin sending 20Hz commands

        Returns:
            observation: Initial state
            info: Additional information
        """
        super().reset(seed=seed)

        # Drift check: restart SITL if position drifted > 300m from origin
        DRIFT_LIMIT = 2000.0
        pos = self.telemetry['position']
        drift = np.sqrt(pos[0]**2 + pos[1]**2)
        if drift > DRIFT_LIMIT:
            print(f"  ⚠️  SITL drift detected: {drift:.0f}m from origin — restarting SITL...")
            self._restart_sitl()

        # Reset episode counters
        self.current_step = 0
        self.episode_reward = 0.0
        self.prev_action = np.zeros(4)
        self.prev_goal_dist = None
        self._approached_goal = False
        self._goal_reached_flag = False
        self._target_pos = np.zeros(3)  # reset; will be set after M99_REF capture
        self._vel_cmd_prev = np.zeros(3)  # low-pass filter state for vel_cmd

        # Goal and obstacles are placed after M99_REF capture below,
        # relative to the drone's actual episode-start position.

        # Refresh data streams each episode (streams degrade over time)
        self.request_data_streams()

        # Disarm if currently armed
        if self.telemetry['armed']:
            self.disarm()
            time.sleep(0.2)

        # Step 1: Disable pre-arm checks
        self.mav.mav.param_set_send(
            self.mav.target_system, self.mav.target_component,
            b'ARMING_CHECK', 0,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32
        )
        ack_deadline = time.time() + 1.0
        while time.time() < ack_deadline:
            msg = self.mav.recv_match(blocking=True, timeout=0.2)
            if msg and msg.get_type() == 'PARAM_VALUE':
                if msg.param_id.strip('\x00') == 'ARMING_CHECK':
                    break

        # Step 2: Switch to GUIDED while disarmed
        self.set_mode('GUIDED')
        time.sleep(0.1)

        # Step 3: Wait for GPS fix + EKF convergence.
        # Skipped on first reset — connect() already waited for the EKF token.
        # Re-checked after SITL drift-restart (when _ekf_ready is cleared).
        if not self._ekf_ready:
            print("  Waiting for GPS fix + EKF convergence...")
            ekf_deadline = time.time() + 30.0
            gps_ok = False
            ekf_ok = False
            gps_count = 0
            while time.time() < ekf_deadline and not (gps_ok and ekf_ok):
                msg = self.mav.recv_match(blocking=True, timeout=0.5)
                if msg is None:
                    continue
                mt = msg.get_type()
                if mt == 'GPS_RAW_INT' and msg.fix_type >= 3 and msg.satellites_visible >= 6:
                    gps_ok = True
                elif mt == 'EKF_STATUS_REPORT':
                    has_vel = bool(msg.flags & 0x002)
                    has_pos = bool(msg.flags & 0x010)
                    if gps_ok and has_vel and has_pos:
                        ekf_ok = True
            if not gps_ok:
                print("  WARNING: GPS fix not confirmed, proceeding anyway")
            if not ekf_ok:
                print("  WARNING: EKF not validated, proceeding anyway")
        self._ekf_ready = True  # mark ready for subsequent resets

        # Step 3b: Reset HOME to current GPS position so LOCAL_POSITION_NED starts
        # at (0,0,0) after SITL restart. Must be done after GPS lock is confirmed.
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            0,   # confirmation
            1,   # use_current=1 → set home to current GPS position
            0, 0, 0, 0, 0, 0
        )
        # Wait for SET_HOME ACK — required before arming ("AHRS: waiting for home")
        home_deadline = time.time() + 3.0
        while time.time() < home_deadline:
            msg = self.mav.recv_match(blocking=True, timeout=0.3)
            if msg and msg.get_type() == 'COMMAND_ACK':
                if msg.command == mavutil.mavlink.MAV_CMD_DO_SET_HOME:
                    break

        # Step 4: Arm (force arm, magic 2989)
        self.arm()
        time.sleep(0.1)

        # Step 5: Takeoff to 45m in GUIDED
        self.takeoff(45.0)
        t_takeoff = time.time()
        fallback_pos = None
        while time.time() - t_takeoff < 60.0:
            self.update_telemetry()
            alt = -self.telemetry['position'][2]
            if alt >= 43.0:
                fallback_pos = self.telemetry['position'].copy()
                print(f"  Takeoff complete at {alt:.2f}m")
                break
            time.sleep(0.5)

        # Step 5b: Wait for GUIDED hover to stabilize (low speed) BEFORE switching to Mode 99.
        # GUIDED's position controller handles deceleration safely; Mode 99 LQR cannot.
        print("  Waiting for GUIDED hover stabilization...")
        hold_pos = fallback_pos if fallback_pos is not None else np.zeros(3)
        guided_stable = 0
        for _ in range(800):  # up to 40s real time (200s sim at speedup 5)
            self.update_telemetry()
            pos = self.telemetry['position']
            vel = self.telemetry['velocity']
            horiz_speed = np.sqrt(vel[0]**2 + vel[1]**2)
            # Send position hold in GUIDED mode to actively decelerate
            self.send_position_target(position=hold_pos)
            if pos[2] < -35.0 and abs(vel[2]) < 0.3 and horiz_speed < 0.5:
                guided_stable += 1
                if guided_stable >= 10:
                    fallback_pos = pos.copy()
                    print(f"  GUIDED stable: alt={-pos[2]:.1f}m horiz={horiz_speed:.2f}m/s")
                    break
            else:
                guided_stable = 0
            time.sleep(0.05 / self.time_scale)
        else:
            print(f"  GUIDED stabilization timeout (horiz={horiz_speed:.1f}m/s alt={-pos[2]:.1f}m)")
            # If drone is moving too fast to stabilize, restart SITL and retry reset
            if horiz_speed > 3.0:
                print(f"  ⚠️  Speed too high ({horiz_speed:.1f}m/s) — restarting SITL and retrying")
                self._restart_sitl()
                return self.reset(seed=seed, options=options)

        # Step 6: Switch to Mode 99
        self.mav.mav.set_mode_send(
            self.mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            99
        )

        # Step 7: Capture M99_REF_* while sending hold commands.
        # With --speedup 5, 10s real = 50s sim >> 12s companion timeout unless we
        # actively send SET_POSITION_TARGET to keep Mode 99 alive.
        ref_n, ref_e, ref_d = None, None, None
        mode99_ok = False
        ref_deadline = time.time() + 5.0
        hold_pos = fallback_pos if fallback_pos is not None else np.zeros(3)
        last_cmd_t = 0.0
        print("  Waiting for Mode 99 + M99_REF_* reference...")
        while time.time() < ref_deadline:
            if mode99_ok and ref_n is not None and ref_e is not None and ref_d is not None:
                break
            # Send hold command every 0.1s real time (~0.5s sim at speedup 5)
            # to prevent the 12s companion timeout from triggering LAND failsafe.
            now = time.time()
            if now - last_cmd_t >= 0.1:
                self.send_position_target(position=hold_pos)
                last_cmd_t = now
            msg = self.mav.recv_match(blocking=True, timeout=0.05)
            if msg is None:
                continue
            mt = msg.get_type()
            if mt == 'HEARTBEAT' and msg.get_srcSystem() == self.mav.target_system:
                if msg.custom_mode == 99:
                    mode99_ok = True
            elif mt == 'NAMED_VALUE_FLOAT':
                name = msg.name.rstrip('\x00')
                if name == 'M99_REF_N':
                    ref_n = msg.value
                elif name == 'M99_REF_E':
                    ref_e = msg.value
                elif name == 'M99_REF_D':
                    ref_d = msg.value

        # Fall back to last known position if M99_REF not received
        if None in (ref_n, ref_e, ref_d):
            print("  WARNING: M99_REF not received, using fallback position")
            p = fallback_pos if fallback_pos is not None else np.zeros(3)
            ref_n, ref_e, ref_d = float(p[0]), float(p[1]), float(p[2])

        # Store Mode 99 reference so step() can use it
        self._mode99_ref = np.array([ref_n, ref_e, ref_d], dtype=np.float32)
        print(f"  Mode 99 reference: N={ref_n:.2f} E={ref_e:.2f} D={ref_d:.2f} (alt={-ref_d:.2f}m)")

        # Wait briefly for Mode 99 to confirm it's running and update ref altitude.
        # (Drone should already be stable from GUIDED pre-stabilization above.)
        print("  Waiting for Mode 99 altitude confirmation...")
        stable_count = 0
        for _ in range(100):  # max 5s real time
            self.update_telemetry()
            pos = self.telemetry['position']
            vel = self.telemetry['velocity']
            horiz_speed = np.sqrt(vel[0]**2 + vel[1]**2)
            if pos[2] < -5.0 and abs(vel[2]) < 0.3 and horiz_speed < 1.0:
                stable_count += 1
                if stable_count >= 5:
                    self._mode99_ref[2] = pos[2]
                    ref_d = pos[2]
                    print(f"  Mode 99 stable: alt={-pos[2]:.1f}m horiz={horiz_speed:.2f}m/s")
                    break
            else:
                stable_count = 0
            self.send_position_target(position=np.array([pos[0], pos[1], ref_d]))
            time.sleep(0.05 / self.time_scale)
        else:
            if pos[2] < -5.0:
                self._mode99_ref[2] = pos[2]
                ref_d = pos[2]
            print(f"  Stabilization timeout, using alt={-ref_d:.1f}m")

        # Initialize integrated position target to drone's actual start position
        start_pos = self.telemetry['position']
        self._target_pos = np.array([start_pos[0], start_pos[1], start_pos[2]], dtype=np.float32)

        # Set goal and obstacles relative to drone's episode-start NE position.
        # This keeps training consistent regardless of how far the drone drifted
        # between episodes during LAND mode.
        origin_ne = self._mode99_ref[:2]
        origin_d = self._mode99_ref[2]  # drone's altitude in NED (e.g. -43m)
        # Place goal at a random direction and random distance from the drone.
        # Previously goal was always placed in +N direction → directional bias
        # and large goal distances when drone drifted in E/W direction.
        angle = self.np_random.uniform(0, 2 * np.pi)
        dist  = self.np_random.uniform(self.goal_dist_min, self.goal_dist_max)
        self.goal_position = np.array([
            origin_ne[0] + dist * np.cos(angle),
            origin_ne[1] + dist * np.sin(angle),
            origin_d + self.np_random.uniform(-5, 5)  # same altitude ±5m
        ], dtype=np.float32)
        self.initial_goal_dist = float(dist)  # 2D distance to goal at episode start
        self._obstacles = []
        if self.enable_obstacles:
            num_obstacles = self.np_random.integers(3, 9)
            for _ in range(num_obstacles):
                self._obstacles.append(self._sample_obstacle(origin_ne))

        # Update telemetry and start position
        self.update_telemetry()
        self.start_position = self.telemetry['position'].copy()

        obs = self.get_observation()
        info = self.get_info()

        return obs, info

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        """
        Execute one step in the environment

        Args:
            action: [vel_toward_goal, vel_lateral, vel_D, yaw_rate]  — goal-relative velocity + yaw rate

        Returns:
            observation: Current state
            reward: Reward for this step
            terminated: Episode ended (goal reached or crashed)
            truncated: Episode truncated (max steps)
            info: Additional information
        """
        self.current_step += 1

        # Clip action to valid range
        action = np.clip(action, self.action_space.low, self.action_space.high)

        # Outside goal radius: enforce minimum forward speed to prevent hovering
        # Inside goal radius: allow deceleration to 0 so agent can learn to stop
        goal_dist_now = np.linalg.norm(self.goal_position - self.telemetry['position'])
        if goal_dist_now > self.goal_radius:
            action[0] = max(action[0], self.min_vel_fwd)

        # --- Goal-relative → NED coordinate conversion ---
        # action[0] = vel_toward_goal: speed in the direction of the goal (always ≥ 0)
        # action[1] = vel_lateral:     speed perpendicular to goal direction (left/right)
        # action[2] = vel_D:           vertical speed (NED: + = descend)
        # action[3] = yaw_rate
        #
        # Compute unit vector toward goal in horizontal (NE) plane
        current_pos = self.telemetry['position']
        goal_rel_h = self.goal_position[:2] - current_pos[:2]  # [dN, dE]
        goal_dist_h = np.linalg.norm(goal_rel_h)
        if goal_dist_h > 1e-3:
            goal_dir = goal_rel_h / goal_dist_h         # unit vector toward goal
        else:
            goal_dir = np.array([1.0, 0.0])             # default North when already at goal
        lateral_dir = np.array([-goal_dir[1], goal_dir[0]])  # 90° left of goal dir

        vel_toward = float(action[0])
        vel_lat    = float(action[1])
        vel_cmd = np.array([
            vel_toward * goal_dir[0] + vel_lat * lateral_dir[0],   # vel_N
            vel_toward * goal_dir[1] + vel_lat * lateral_dir[1],   # vel_E
            float(action[2])                                         # vel_D
        ], dtype=np.float32)

        # Safety cap: prevent diagonal combination from exceeding MAX_VEL_H.
        # e.g. toward=4.0 + lateral=4.0 would give 5.66 m/s if not clipped.
        MAX_VEL_H = 3.5
        h_mag = np.sqrt(vel_cmd[0]**2 + vel_cmd[1]**2)
        if h_mag > MAX_VEL_H:
            scale = MAX_VEL_H / h_mag
            vel_cmd[0] *= scale
            vel_cmd[1] *= scale

        # Position target = current position (LOOKAHEAD=0).
        # With LOOKAHEAD>0 the position error was always non-zero, creating a
        # persistent forward-drive even at target speed. LOOKAHEAD=0 means the
        # LQR only sees a velocity error, which naturally drives speed toward vel_ref.
        self._target_pos = current_pos.copy()

        # Altitude clamping: keep within ±10m of takeoff altitude
        ref_d = self._mode99_ref[2]
        self._target_pos[2] = np.clip(self._target_pos[2], ref_d - 10.0, ref_d + 10.0)

        # Rate limiter: cap vel_cmd change per step to prevent excessive tilt.
        # 0.3 m/s/step @ 20Hz = 6 m/s² → tilt ≈ 31° (safely below 40° TILT limit)
        # Replaces LP filter — rate limit is a hard physical constraint, LP filter is not.
        MAX_VEL_RATE = 0.3  # m/s per step
        delta = vel_cmd - self._vel_cmd_prev
        delta_clipped = np.clip(delta, -MAX_VEL_RATE, MAX_VEL_RATE)
        vel_cmd_limited = self._vel_cmd_prev + delta_clipped
        self._vel_cmd_prev = vel_cmd_limited.copy()

        # Tilt recovery: scale down vel_cmd when tilt is large so LQR can
        # recover attitude before chasing velocity again.
        # tilt ≤ 15°: full command / tilt ≥ 35°: zero command (hover)
        # _vel_cmd_prev is updated with the scaled value so rate limiter
        # ramps up from zero after recovery — not from the pre-tilt speed.
        att = self.telemetry['attitude']
        tilt_deg = np.degrees(np.sqrt(att[0]**2 + att[1]**2))
        tilt_scale = float(np.clip(1.0 - (tilt_deg - 10.0) / 20.0, 0.0, 1.0))

        vel_ref = vel_cmd_limited * tilt_scale
        self._vel_cmd_prev = vel_ref.copy()  # rate limiter resumes from scaled value
        self.send_position_target(
            position=self._target_pos.copy(),
            velocity=vel_ref,
            yaw_rate=action[3]
        )

        # Debug: print action and vel_cmd for first 5 steps and every 50 steps
        if self.current_step <= 5 or self.current_step % 50 == 0:
            print(f"  [dbg step{self.current_step:3d}] act=[{action[0]:.2f},{action[1]:.2f},{action[2]:.2f},{action[3]:.2f}] vel_cmd=[{vel_cmd[0]:.2f},{vel_cmd[1]:.2f},{vel_cmd[2]:.2f}] goal_dir=[{goal_dir[0]:.2f},{goal_dir[1]:.2f}]")

        # Log altitude every 100 steps to track trajectory
        if self.current_step % 100 == 0:
            pos = self.telemetry['position']
            vel = self.telemetry['velocity']
            mode = self.telemetry['mode']
            lqi_thrust = self.telemetry['lqi_diag'].get('LQI_Thrust', float('nan'))
            att = self.telemetry['attitude']
            tilt_deg = np.degrees(np.sqrt(att[0]**2 + att[1]**2))
            speed_h = np.sqrt(vel[0]**2 + vel[1]**2)
            speed_3d = np.linalg.norm(vel)
            tgt_err = np.linalg.norm(self._target_pos - pos)
            out_ptch = self.telemetry['lqi_diag'].get('OUT_ptch', float('nan'))
            out_roll = self.telemetry['lqi_diag'].get('OUT_roll', float('nan'))
            m_pitch  = self.telemetry['lqi_diag'].get('LQI_M_ptch', float('nan'))
            print(f"  [step {self.current_step:4d}] alt={-pos[2]:.1f}m spd_h={speed_h:.1f}m/s spd={speed_3d:.1f}m/s tilt={tilt_deg:.1f}° thrust={lqi_thrust:.1f}N tgt_err={tgt_err:.1f}m mode={mode} out_ptch={out_ptch:.3f} out_roll={out_roll:.3f} M_pitch={m_pitch:.3f}")

        # Wait for control period (20Hz = 50ms)
        time.sleep(0.05 / self.time_scale)

        # Update telemetry
        self.update_telemetry()

        # Get observation
        obs = self.get_observation()

        # Calculate reward
        reward = self.calculate_reward(obs, action)
        self.episode_reward += reward

        # Check termination conditions
        terminated = self.is_terminated() or self._goal_reached_flag
        goal_dist_now = np.linalg.norm(self.goal_position - self.telemetry['position'])
        if goal_dist_now < self.goal_radius * 1.0:
            self._approached_goal = True
        passed_goal = (self._approached_goal and
                       goal_dist_now > self.goal_radius * 1.5)
        truncated = self.current_step >= self.max_steps or passed_goal

        # Log episode end reason
        if terminated or truncated:
            goal_relative = self.goal_position - self.telemetry['position']
            goal_dist = np.linalg.norm(goal_relative)
            pos = self.telemetry['position']
            vel = self.telemetry['velocity']
            speed = np.linalg.norm(vel)
            goal = self.goal_position
            att_end = self.telemetry['attitude']
            tilt_at_end = np.sqrt(att_end[0]**2 + att_end[1]**2)
            print(f"  pos=({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}) "
                  f"vel=({vel[0]:.1f}, {vel[1]:.1f}, {vel[2]:.1f}) speed={speed:.1f}m/s "
                  f"goal=({goal[0]:.1f}, {goal[1]:.1f}, {goal[2]:.1f})")
            if goal_dist < self.goal_radius:
                print(f"🎯 GOAL REACHED! step={self.current_step} reward={self.episode_reward:.1f}")
            elif np.any(self.telemetry['obstacles'] < 1.0):
                print(f"💥 CRASH! step={self.current_step} reward={self.episode_reward:.1f}")
            elif self.telemetry['position'][2] > -5.0:
                print(f"🌍 GROUND! step={self.current_step} reward={self.episode_reward:.1f}")
            elif tilt_at_end > np.radians(40.0):
                print(f"↗️ HIGH TILT (continued) step={self.current_step} reward={self.episode_reward:.1f} tilt={np.degrees(tilt_at_end):.1f}°")
            elif not self.telemetry['armed']:
                print(f"⚠️ DISARMED! step={self.current_step} reward={self.episode_reward:.1f}")
            elif passed_goal:
                print(f"🏃 PASSED GOAL! step={self.current_step} reward={self.episode_reward:.1f} goal_dist={goal_dist:.1f}m (threshold={self.goal_radius * 1.5:.1f}m)")
            else:
                print(f"⏱️ TIMEOUT! step={self.current_step} reward={self.episode_reward:.1f} goal_dist={goal_dist:.1f}m")

        # Additional info
        info = self.get_info()

        # Store previous action and goal dist for next step
        self.prev_action = action.copy()
        self.prev_goal_dist = np.linalg.norm(self.goal_position - self.telemetry['position'])

        return obs, reward, terminated, truncated, info

    def get_observation(self) -> np.ndarray:
        """
        Get current observation from telemetry

        Returns:
            25D observation vector
        """
        # Compute virtual LiDAR distances from current drone position
        self.telemetry['obstacles'] = self._compute_obstacle_distances()

        # Goal relative position
        goal_relative = self.goal_position - self.telemetry['position']

        # Construct observation
        obs = np.concatenate([
            self.telemetry['velocity'],      # 3  [0:3]
            self.telemetry['attitude'],      # 3  [3:6]
            self.telemetry['rates'],         # 3  [6:9]
            [self.telemetry['battery']],     # 1  [9]
            self.telemetry['gps'],           # 4  [10:14]
            self.telemetry['obstacles'],     # 6  [14:20]
            goal_relative                     # 3  [20:23]
        ])

        return obs.astype(np.float32)

    def calculate_reward(self, obs: np.ndarray, action: np.ndarray) -> float:
        """
        Calculate reward for current step.

        Responsibility split:
          RPi (this reward):  goal reaching, obstacle avoidance, altitude maintenance, stable flight
          Mode 99 (LQR):      attitude stability, tilt control, speed regulation — NOT rewarded here

        Phase 0 curriculum — teach goal-directed flight:
            1. Goal-directed travel     +10 * dist_toward_goal_per_step → reward vel_toward_goal action
            2. Goal reached bonus       +1000 on arrival
            3. Progress bonus           reward approaching goal each step
            4. Obstacle/crash penalty   safety constraints
            5. Altitude maintenance     keep near takeoff altitude
        """
        reward = 0.0

        velocity = obs[0:3]
        goal_relative = obs[-3:]
        goal_dist = np.linalg.norm(goal_relative)
        obstacles = obs[14:20]

        # 1. Goal-directed travel bonus: bilateral (approach → bonus, retreat → penalty)
        #    vel_toward_goal = velocity · goal_dir (positive = toward goal, negative = away)
        #    dist_toward_goal = vel_toward_goal * dt (meters closed per step)
        #    × 10 coefficient → at 1.5 m/s toward goal: +0.75/step, at 1.5 m/s away: -0.75/step
        vel_toward_goal = 0.0
        speed_h = np.linalg.norm(velocity[:2])
        if goal_dist > 1e-6:
            goal_dir_h = goal_relative[:2] / (np.linalg.norm(goal_relative[:2]) + 1e-6)
            vel_toward_goal = velocity[0] * goal_dir_h[0] + velocity[1] * goal_dir_h[1]
            dist_toward_goal = vel_toward_goal * 0.05  # positive = closing, negative = retreating
            reward += 10.0 * dist_toward_goal  # bilateral: retreating → penalty

        # 1b. Alignment bonus/penalty: reward goal-directed flight, penalize lateral drift
        #     alignment = vel_toward_goal / speed_h: +1=straight, 0=perpendicular, -1=retreating
        #     lateral_speed = sqrt(speed_h² - vel_toward_goal²): explicit lateral penalty
        if speed_h > 0.1:
            alignment = vel_toward_goal / speed_h
            reward += 2.0 * alignment                              # straight → +2.0, sideways → 0, retreat → -2.0
            vel_lateral_actual = np.sqrt(max(0.0, speed_h**2 - vel_toward_goal**2))
            reward -= 3.0 * vel_lateral_actual                     # lateral speed → explicit penalty

        # 3. Goal reached bonus (one-time: fires on first entry into goal_radius)
        if goal_dist < self.goal_radius and not self._goal_reached_flag:
            reward += 1000.0
            self._goal_reached_flag = True

        # 4. Progress bonus (approaching goal each step) — bilateral
        if self.prev_goal_dist is not None:
            dist_improvement = self.prev_goal_dist - goal_dist
            reward += 2.0 * dist_improvement  # bilateral: approaching → bonus, retreating → penalty

        # 5. Obstacle proximity penalty
        for d in obstacles:
            if d < 5.0:
                reward -= 2.0 * np.exp(-d)

        # 6. Crash penalty
        if np.any(obstacles < 1.0):
            reward -= 500.0

        # 7. Altitude maintenance (±1m dead zone, coefficient 0.1)
        current_alt = -self.telemetry['position'][2]
        target_alt  = -self._mode99_ref[2]
        alt_error = max(0.0, abs(current_alt - target_alt) - 1.0)
        reward -= 0.1 * alt_error

        # 8. Directional speed bonus based on actual velocity toward goal — bilateral
        #    action[0] now has lower bound 1.5 m/s so the drone always moves toward goal.
        #    This reward still encourages going faster.
        reward += 0.3 * vel_toward_goal

        # 8c. Lateral and yaw-rate penalty: penalize unnecessary sideways motion and spinning
        #     lateral: orthogonal to goal → never reduces vel_toward_goal, so "free" without penalty
        #     yaw_rate: causes heading drift → LQR load increases → trajectory deviates + TILT risk
        #     -2.0 × |lateral|  → at max ±1.0: -2.0/step
        #     -2.0 × |yaw_rate| → at max ±0.3: -0.6/step
        reward -= 2.0 * abs(action[1])
        reward -= 2.0 * abs(action[3])

        # 8b. Deceleration reward near goal
        #     Within 1.0× goal_radius (10m), penalize high approach speed to prevent overshoot.
        #     Smoothly increases as drone approaches: coefficient ramps from 0 → 1.0
        #     1.0× (not 1.5×) so decel zone doesn't overlap with goal_min when goal_min=15m.
        decel_threshold = self.goal_radius * 1.0
        if goal_dist < decel_threshold:
            proximity = 1.0 - goal_dist / decel_threshold  # 0 at threshold, 1 at goal
            reward -= 1.0 * proximity * max(0.0, vel_toward_goal - 1.0)

        # 8c. Tilt penalty: penalize high tilt outside goal (goal interior exempt for braking)
        #     Max decel (0.3 m/s/step) = tilt ~31°, so goal braking would be penalized unfairly.
        #     Threshold 25° matches normal flight at MAX_VEL=3.5 m/s.
        att = self.telemetry['attitude']
        tilt_deg = np.degrees(np.sqrt(att[0]**2 + att[1]**2))
        if tilt_deg > 25 and goal_dist >= self.goal_radius:
            reward -= 0.5 * (tilt_deg - 25)  # e.g. tilt=30° → -2.5/step, tilt=35° → -5.0/step

        # 9. Time penalty: discourage hovering in place
        #    -1.0/step × 1500steps = -1500 → strong incentive for high-speed goal approach
        reward -= 1.0

        # 10. Distance penalty: penalize being far from goal each step
        #     -0.05 × goal_dist/step → at 10m: -0.5/step, at 5m: -0.25/step
        #     Makes hovering far from goal very costly; incentivizes closing distance quickly
        reward -= 0.05 * goal_dist

        # 11. Hovering penalty: discourage stationary flight (disabled inside goal)
        #     -1.0 × max(0, 1.0 - spd_h) → at 0 m/s: -1.0/step, at 1 m/s: 0/step
        #     Combined with time+dist penalty: hovering = -2.55/step, 2 m/s toward goal = +0.3/step
        #     Disabled inside goal_radius so agent is not penalized for decelerating after arrival.
        spd_h = np.sqrt(velocity[0]**2 + velocity[1]**2)
        if goal_dist >= self.goal_radius:
            reward -= 1.0 * max(0.0, 1.0 - spd_h)

        # 12. Early arrival bonus: reward reaching goal faster
        #     +500 × (1 - step/max_steps) → at step 300: +400, at step 1200: +100
        #     Rewards high-speed goal approach vs slow crawl
        if goal_dist < self.goal_radius:
            early_bonus = 500.0 * (1.0 - self.current_step / self.max_steps)
            reward += early_bonus


        return reward

    def is_terminated(self) -> bool:
        """Check if episode should terminate"""
        # Goal reached
        goal_relative = self.goal_position - self.telemetry['position']
        if np.linalg.norm(goal_relative) < self.goal_radius:
            return True

        # Crashed (any obstacle < 1m)
        if np.any(self.telemetry['obstacles'] < 1.0):
            return True

        # Ground contact (altitude below 5m)
        if self.telemetry['position'][2] > -5.0:
            return True

        # Excessive tilt (> 40°): unstable, end episode
        att = self.telemetry['attitude']
        tilt = np.sqrt(att[0]**2 + att[1]**2)
        if tilt > np.radians(40.0):
            return True

        # Disarmed unexpectedly
        if not self.telemetry['armed']:
            return True

        return False

    def get_info(self) -> Dict[str, Any]:
        """Get additional information"""
        goal_relative = self.goal_position - self.telemetry['position']

        goal_dist = np.linalg.norm(goal_relative)
        return {
            'goal_distance': goal_dist,
            'goal_reached': goal_dist < self.goal_radius,
            'min_obstacle_dist': np.min(self.telemetry['obstacles']),
            'episode_reward': self.episode_reward,
            'current_step': self.current_step,
            'position': self.telemetry['position'].copy(),
            'velocity': self.telemetry['velocity'].copy()
        }

    def _sample_obstacle(self, origin_ne: np.ndarray = None) -> tuple:
        """
        Sample a random vertical-column box obstacle relative to origin_ne.

        origin_ne: [N, E] of the drone's episode-start position. Obstacles are
                   placed relative to this so they remain between the drone and
                   its goal regardless of where SITL drifted between episodes.

        Returns:
            (center, half_size) both as np.ndarray[3] in NED meters
        """
        if origin_ne is None:
            origin_ne = np.zeros(2)
        if self.mission_type == 'obstacle_avoidance':
            goal_n_rel = self.goal_position[0] - origin_ne[0]
            n = float(self.np_random.uniform(5.0, max(goal_n_rel - 5.0, 6.0))) + origin_ne[0]
            e = float(self.np_random.uniform(-8.0, 8.0)) + origin_ne[1]
        else:
            n = float(self.np_random.uniform(-45.0, 45.0)) + origin_ne[0]
            e = float(self.np_random.uniform(-45.0, 45.0)) + origin_ne[1]
        # Vertical column spanning drone's flight altitude range
        origin_d = self._mode99_ref[2]  # e.g. -43m
        center = np.array([n, e, origin_d], dtype=np.float32)
        half_size = np.array([1.0, 1.0, 10.0], dtype=np.float32)  # ±10m vertically
        return center, half_size

    def _compute_obstacle_distances(self) -> np.ndarray:
        """
        Cast 6 rays from current drone position and return distances to the
        nearest virtual obstacle surface in each direction.

        Returns:
            distances[6]: [front, back, right, left, up, down], capped at _LIDAR_RANGE
        """
        pos = self.telemetry['position']
        dists = np.full(6, self._LIDAR_RANGE, dtype=np.float32)
        for center, half_size in self._obstacles:
            box_min = center - half_size
            box_max = center + half_size
            for i, direction in enumerate(self._LIDAR_DIRS):
                d = self._ray_aabb_dist(pos, direction, box_min, box_max)
                if d < dists[i]:
                    dists[i] = d
        return dists

    @staticmethod
    def _ray_aabb_dist(origin: np.ndarray, direction: np.ndarray,
                       box_min: np.ndarray, box_max: np.ndarray) -> float:
        """
        Ray-AABB intersection distance.

        Returns 0.0 if origin is inside the box (collision),
        inf if ray misses the box entirely, otherwise the entry distance.
        """
        t_min = -np.inf
        t_max = np.inf
        for i in range(3):
            if abs(direction[i]) < 1e-10:
                if origin[i] < box_min[i] or origin[i] > box_max[i]:
                    return np.inf
            else:
                t1 = (box_min[i] - origin[i]) / direction[i]
                t2 = (box_max[i] - origin[i]) / direction[i]
                t_min = max(t_min, min(t1, t2))
                t_max = min(t_max, max(t1, t2))
        if t_min > t_max or t_max < 0:
            return np.inf
        if t_min < 0:
            return 0.0  # Origin inside box = collision
        return float(t_min)

    def update_telemetry(self):
        """Update telemetry from MAVLink messages"""
        # Receive all pending messages
        while True:
            msg = self.mav.recv_match(blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()

            if msg_type == 'LOCAL_POSITION_NED':
                self.telemetry['position'] = np.array([msg.x, msg.y, msg.z])
                self.telemetry['velocity'] = np.array([msg.vx, msg.vy, msg.vz])

            elif msg_type == 'ATTITUDE':
                self.telemetry['attitude'] = np.array([msg.roll, msg.pitch, msg.yaw])
                self.telemetry['rates'] = np.array([msg.rollspeed, msg.pitchspeed, msg.yawspeed])

            elif msg_type == 'SYS_STATUS':
                self.telemetry['battery'] = msg.voltage_battery / 1000.0  # mV to V

            elif msg_type == 'GPS_RAW_INT':
                self.telemetry['gps'] = np.array([
                    msg.lat / 1e7,
                    msg.lon / 1e7,
                    msg.alt / 1000.0,
                    msg.satellites_visible
                ])

            elif msg_type == 'HEARTBEAT':
                self.telemetry['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.telemetry['mode'] = msg.custom_mode

            elif msg_type == 'NAMED_VALUE_FLOAT':
                name = msg.name.rstrip('\x00')
                self.telemetry['lqi_diag'][name] = msg.value

            elif msg_type == 'STATUSTEXT':
                text = msg.text.rstrip('\x00')
                if text.startswith('M99 ') or text.startswith('SMARTPHOTO99'):
                    print(f"    [SITL] {text}")

    def send_position_target(
        self,
        position: np.ndarray,
        velocity: Optional[np.ndarray] = None,
        yaw: float = 0.0,
        yaw_rate: float = 0.0
    ):
        """
        Send position/velocity target to Mode 99.

        Args:
            position: Target position [x, y, z] in NED (meters)
            velocity: Target velocity [vx, vy, vz] in NED (m/s) (optional)
            yaw: Target yaw (radians) — ignored by Mode 99 (use yaw_rate instead)
            yaw_rate: Target yaw rate (rad/s)
        """
        if velocity is None:
            velocity = np.zeros(3)

        # Type mask: use pos + vel + yaw_rate; ignore yaw and acceleration
        # bit 6,7,8 = ignore acc_x/y/z, bit 10 = ignore yaw (use yaw_rate via bit 11=0)
        type_mask = 0b0000_0101_1100_0000  # 0x05C0

        self.mav.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            position[0], position[1], position[2],  # x, y, z (NED, meters)
            velocity[0], velocity[1], velocity[2],  # vx, vy, vz (NED, m/s)
            0.0, 0.0, 0.0,                          # acceleration (ignored)
            yaw,
            yaw_rate
        )

    def arm(self):
        """Arm the copter (force arm, magic 2989). Retries every 2s up to 15s.
        Monitors both COMMAND_ACK (arm accepted/rejected) and HEARTBEAT (armed bit).
        """
        arm_deadline = time.time() + 15.0
        last_send = 0.0
        while time.time() < arm_deadline:
            # Send arm command every 2 seconds until confirmed
            if time.time() - last_send >= 2.0:
                self.mav.mav.command_long_send(
                    self.mav.target_system, self.mav.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 2989, 0, 0, 0, 0, 0
                )
                last_send = time.time()
            msg = self.mav.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue
            mt = msg.get_type()
            if mt == 'HEARTBEAT':
                if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    self.telemetry['armed'] = True
                    print("✅ Armed")
                    return
            elif mt == 'COMMAND_ACK':
                if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        # ACK received — wait briefly for HEARTBEAT to reflect armed bit
                        ack_deadline = time.time() + 2.0
                        while time.time() < ack_deadline:
                            hb = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
                            if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                                self.telemetry['armed'] = True
                                print("✅ Armed")
                                return
                        self.telemetry['armed'] = True
                        print("✅ Armed (ACK confirmed)")
                        return
                    else:
                        print(f"  Arm rejected (result={msg.result}), retrying...")
                        last_send = 0.0  # force immediate retry
            elif mt == 'STATUSTEXT':
                text = msg.text.strip('\x00').strip()
                if text:
                    print(f"  [SITL] {text}")
        print("⚠️ Arm timeout")

    def disarm(self):
        """Disarm with 5s timeout; force disarm if auto-disarm stalls."""
        self.mav.arducopter_disarm()
        deadline = time.time() + 5.0
        while time.time() < deadline:
            msg = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=0.3)
            if msg is None:
                continue
            if not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                self.telemetry['armed'] = False
                print("  Disarmed")
                return
        # Timeout — force disarm via MAV_CMD (magic 21196)
        print("  Disarm timeout, sending force disarm")
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 21196, 0, 0, 0, 0, 0
        )
        time.sleep(1.5)
        self.telemetry['armed'] = False
        print("  Force disarmed")

    def takeoff(self, altitude: float):
        """Takeoff to specified altitude"""
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude
        )
        print(f"✅ Taking off to {altitude}m")

    def set_mode(self, mode: str):
        """Set flight mode"""
        mode_id = self.mav.mode_mapping()[mode]
        self.mav.set_mode(mode_id)
        print(f"✅ Mode set to {mode}")

    def render(self, mode='human'):
        """Render environment (optional)"""
        if mode == 'human':
            print(f"Step: {self.current_step}, "
                  f"Pos: {self.telemetry['position']}, "
                  f"Goal: {self.goal_position}, "
                  f"Reward: {self.episode_reward:.2f}")

    def close(self):
        """Clean up resources"""
        if self.mav:
            self.disarm()
            self.set_mode('STABILIZE')
            self.mav.close()


if __name__ == '__main__':
    # Test environment
    print("Testing ArduPilot Mode 99 Gym Environment")
    print("Make sure SITL is running: ./Tools/autotest/sim_vehicle.py -v ArduCopter")

    env = ArduPilotMode99Env(
        sitl_connection='tcp:127.0.0.1:5760',
        mission_type='obstacle_avoidance',
        max_steps=500
    )

    # Test reset
    obs, info = env.reset()
    print(f"Initial observation shape: {obs.shape}")
    print(f"Initial info: {info}")

    # Test a few steps with random actions
    for i in range(10):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        env.render()

        if terminated or truncated:
            break

    env.close()
    print("✅ Environment test complete")
