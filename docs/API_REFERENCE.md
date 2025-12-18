# API Reference

Complete API documentation for the Autonomous Drone Simulation Environment.

---

## Table of Contents

1. [Gym Environments](#gym-environments)
2. [Controllers](#controllers)
3. [Sensors](#sensors)
4. [Path Planning Algorithms](#path-planning-algorithms)
5. [Utilities](#utilities)
6. [Configuration](#configuration)

---

## Gym Environments

### BaseDroneEnv

Base class for all drone environments.

#### Import
```python
from drone_gym.envs import BaseDroneEnv
```

#### Constructor

```python
BaseDroneEnv(
    mavlink_connection: str = "udp:127.0.0.1:14550",
    airsim_host: str = "127.0.0.1",
    control_freq: float = 50.0,
    max_episode_steps: int = 1000,
    time_step: float = 0.02,
    safety_distance: float = 3.0,
    no_fly_zones: Optional[List[Tuple[np.ndarray, float]]] = None
)
```

**Parameters:**
- `mavlink_connection` (str): MAVLink connection string
  - Default: `"udp:127.0.0.1:14550"`
  - Format: `"udp:<host>:<port>"` or `"tcp:<host>:<port>"`

- `airsim_host` (str): AirSim server hostname
  - Default: `"127.0.0.1"`

- `control_freq` (float): Control loop frequency in Hz
  - Default: `50.0` (20ms cycle time)
  - Range: `10.0` - `400.0`

- `max_episode_steps` (int): Maximum steps per episode
  - Default: `1000`
  - Range: `100` - `10000`

- `time_step` (float): Time step in seconds
  - Default: `0.02` (20ms)
  - Automatically calculated from control_freq: `1.0 / control_freq`

- `safety_distance` (float): Minimum safety distance from obstacles (meters)
  - Default: `3.0`
  - Range: `0.5` - `10.0`

- `no_fly_zones` (List[Tuple[np.ndarray, float]], optional): List of no-fly zones
  - Format: `[(center, radius), ...]`
  - Example: `[(np.array([10, 20, -15]), 5.0)]`

#### Methods

##### `connect(timeout: float = 10.0) -> bool`

Connect to drone systems (MAVLink and AirSim).

**Parameters:**
- `timeout` (float): Connection timeout in seconds

**Returns:**
- `bool`: True if all connections successful

**Example:**
```python
env = BaseDroneEnv()
if env.connect(timeout=30.0):
    print("Connected successfully")
else:
    print("Connection failed")
```

##### `reset() -> np.ndarray`

Reset environment to initial state.

**Returns:**
- `np.ndarray`: Initial observation (888D vector)

**Example:**
```python
obs = env.reset()
print(f"Observation shape: {obs.shape}")  # (888,)
```

##### `step(action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict]`

Execute one time step.

**Parameters:**
- `action` (np.ndarray): Action vector [vx, vy, vz, yaw_rate]
  - Shape: `(4,)`
  - vx, vy ∈ [-5.0, 5.0] m/s
  - vz ∈ [-2.0, 2.0] m/s
  - yaw_rate ∈ [-1.0, 1.0] rad/s

**Returns:**
- `observation` (np.ndarray): Next observation (888D)
- `reward` (float): Reward for this step
- `done` (bool): Whether episode is finished
- `info` (dict): Additional information
  - `episode_length` (int): Current episode length
  - `total_reward` (float): Cumulative reward
  - `distance_to_goal` (float): Distance to target (m)
  - `collision` (bool): Whether collision occurred
  - `mission_completed` (bool): Whether mission completed
  - `battery` (float): Battery percentage
  - `termination_reason` (str): Reason for episode end

**Example:**
```python
action = np.array([1.0, 0.0, 0.0, 0.0])  # Move forward at 1 m/s
obs, reward, done, info = env.step(action)

print(f"Reward: {reward:.2f}")
print(f"Distance to goal: {info['distance_to_goal']:.2f}m")

if done:
    print(f"Episode ended: {info['termination_reason']}")
```

##### `render(mode: str = 'rgb_array') -> Optional[np.ndarray]`

Render environment.

**Parameters:**
- `mode` (str): Rendering mode
  - `'rgb_array'`: Return RGB image
  - `'human'`: Display in window

**Returns:**
- `np.ndarray` or `None`: RGB image if mode='rgb_array'

**Example:**
```python
rgb_image = env.render(mode='rgb_array')
if rgb_image is not None:
    print(f"Image shape: {rgb_image.shape}")  # (1080, 1920, 3)
```

##### `get_performance_metrics() -> Dict[str, Any]`

Get control loop performance metrics.

**Returns:**
- `dict`: Performance metrics
  - `control_loop_mean_ms` (float): Mean loop time (ms)
  - `control_loop_std_ms` (float): Std dev loop time (ms)
  - `control_loop_max_ms` (float): Max loop time (ms)
  - `target_loop_time_ms` (float): Target loop time (ms)
  - `meets_timing_requirement` (bool): Whether timing requirement met

**Example:**
```python
metrics = env.get_performance_metrics()
print(f"Mean loop time: {metrics['control_loop_mean_ms']:.2f}ms")
print(f"Meets requirement: {metrics['meets_timing_requirement']}")
```

#### Properties

##### `observation_space`
```python
gym.spaces.Box(low=-np.inf, high=np.inf, shape=(888,), dtype=np.float32)
```

##### `action_space`
```python
gym.spaces.Box(
    low=np.array([-5.0, -5.0, -2.0, -1.0]),
    high=np.array([5.0, 5.0, 2.0, 1.0]),
    dtype=np.float32
)
```

---

### DroneNavEnv

Basic navigation environment with minimal obstacles.

#### Import
```python
from drone_gym.envs import DroneNavEnv
# or
import gym
env = gym.make('DroneNav-v0')
```

#### Features
- Target: Single waypoint 20-50m away
- Obstacles: 0-3 static obstacles
- Success criteria: >99% completion rate

#### Example
```python
import gym
import drone_gym

env = gym.make('DroneNav-v0')
obs = env.reset()

for step in range(500):
    action = env.action_space.sample()
    obs, reward, done, info = env.step(action)

    if done:
        if info['mission_completed']:
            print("✓ Navigation successful")
        break

env.close()
```

---

### DroneObstacleEnv

Obstacle avoidance environment with static and dynamic obstacles.

#### Import
```python
from drone_gym.envs import DroneObstacleEnv
# or
import gym
env = gym.make('DroneObstacle-v0')
```

#### Features
- Target: Single waypoint 30-80m away
- Obstacles: 10-50 static + 5-10 dynamic
- Dynamic obstacles move at 0.5-3 m/s
- Success criteria: >95% collision-free rate

#### Additional Properties

##### `static_obstacles`
```python
List[Tuple[np.ndarray, float]]
```
List of static obstacles (center, radius).

##### `dynamic_obstacles`
```python
List[Tuple[np.ndarray, float]]
```
List of dynamic obstacles (center, radius).

##### `obstacle_velocities`
```python
List[np.ndarray]
```
Velocities of dynamic obstacles.

#### Example
```python
import gym
import drone_gym

env = gym.make('DroneObstacle-v0')
obs = env.reset()

print(f"Static obstacles: {len(env.static_obstacles)}")
print(f"Dynamic obstacles: {len(env.dynamic_obstacles)}")

for step in range(1000):
    action = my_controller.compute_action(obs)
    obs, reward, done, info = env.step(action)

    if done:
        print(f"Collision-free: {not info['collision']}")
        break

env.close()
```

---

### DroneWaypointEnv

Sequential waypoint navigation environment.

#### Import
```python
from drone_gym.envs import DroneWaypointEnv
# or
import gym
env = gym.make('DroneWaypoint-v0')
```

#### Constructor

```python
DroneWaypointEnv(
    num_waypoints: int = 5,
    **kwargs
)
```

**Parameters:**
- `num_waypoints` (int): Number of waypoints in sequence
  - Default: `5`
  - Range: `3` - `20`

#### Features
- Target: Sequential waypoints
- Patterns: line, circle, square, random
- Waypoint accuracy: <2m
- Obstacles: 5-15 static
- Success criteria: >99% completion rate

#### Additional Properties

##### `waypoints`
```python
List[np.ndarray]
```
List of waypoint positions.

##### `current_waypoint_idx`
```python
int
```
Index of current target waypoint.

##### `waypoint_reached_count`
```python
int
```
Number of waypoints reached.

#### Additional Info Keys

- `waypoint_reached` (bool): Whether waypoint was just reached
- `waypoint_index` (int): Index of reached waypoint
- `waypoint_accuracy` (float): Distance from waypoint (m)
- `waypoints_reached` (int): Total waypoints reached
- `total_waypoints` (int): Total waypoints in mission
- `current_waypoint` (int): Current target waypoint index

#### Example
```python
import gym
import drone_gym

env = gym.make('DroneWaypoint-v0')
obs = env.reset()

print(f"Total waypoints: {len(env.waypoints)}")
for i, wp in enumerate(env.waypoints):
    print(f"  Waypoint {i+1}: [{wp[0]:.1f}, {wp[1]:.1f}, {wp[2]:.1f}]")

for step in range(2000):
    action = my_controller.compute_action(obs)
    obs, reward, done, info = env.step(action)

    if 'waypoint_reached' in info and info['waypoint_reached']:
        wp_idx = info['waypoint_index']
        accuracy = info['waypoint_accuracy']
        print(f"✓ Waypoint {wp_idx+1} reached (accuracy: {accuracy:.2f}m)")

    if done:
        reached = info['waypoints_reached']
        total = info['total_waypoints']
        print(f"Mission: {reached}/{total} waypoints reached")
        break

env.close()
```

---

## Controllers

### MAVLinkInterface

Interface for MAVLink communication with ArduPilot SITL.

#### Import
```python
from drone_gym.controllers.mavlink_interface import MAVLinkInterface
```

#### Constructor

```python
MAVLinkInterface(
    connection_string: str = "udp:127.0.0.1:14550",
    baud_rate: int = 115200,
    timeout: float = 5.0,
    target_system: int = 1,
    target_component: int = 1
)
```

**Parameters:**
- `connection_string` (str): MAVLink connection string
- `baud_rate` (int): Baud rate for serial connections
- `timeout` (float): Connection timeout (seconds)
- `target_system` (int): Target system ID
- `target_component` (int): Target component ID

#### Methods

##### `connect() -> bool`

Establish connection to ArduPilot.

**Returns:**
- `bool`: True if connection successful

**Example:**
```python
mavlink = MAVLinkInterface()
if mavlink.connect():
    print("Connected to ArduPilot")
```

##### `get_state() -> Dict[str, Any]`

Get current drone state.

**Returns:**
- `dict`: Current state
  - `position` (np.ndarray): [x, y, z] in NED (m)
  - `velocity` (np.ndarray): [vx, vy, vz] in NED (m/s)
  - `attitude` (np.ndarray): [roll, pitch, yaw] (rad)
  - `angular_velocity` (np.ndarray): [p, q, r] (rad/s)
  - `battery` (float): Battery percentage
  - `armed` (bool): Whether drone is armed
  - `mode` (str): Current flight mode
  - `gps_fix` (int): GPS fix type
  - `heading` (float): Heading in degrees

**Example:**
```python
state = mavlink.get_state()
print(f"Position: {state['position']}")
print(f"Velocity: {state['velocity']}")
print(f"Battery: {state['battery']:.1f}%")
```

##### `arm(timeout: float = 10.0) -> bool`

Arm the drone.

**Parameters:**
- `timeout` (float): Timeout in seconds

**Returns:**
- `bool`: True if armed successfully

**Example:**
```python
if mavlink.arm(timeout=10.0):
    print("Drone armed")
```

##### `disarm(timeout: float = 10.0) -> bool`

Disarm the drone.

##### `set_mode(mode: str, timeout: float = 5.0) -> bool`

Set flight mode.

**Parameters:**
- `mode` (str): Flight mode name
  - `'GUIDED'`: Guided mode (required for velocity commands)
  - `'LOITER'`: Hold position
  - `'RTL'`: Return to launch
  - `'LAND'`: Land at current position
  - `'STABILIZE'`: Manual stabilization

**Example:**
```python
mavlink.set_mode('GUIDED')
```

##### `takeoff(altitude: float, timeout: float = 30.0) -> bool`

Takeoff to specified altitude.

**Parameters:**
- `altitude` (float): Target altitude in meters
- `timeout` (float): Timeout in seconds

**Returns:**
- `bool`: True if takeoff successful

**Example:**
```python
if mavlink.takeoff(altitude=10.0):
    print("Takeoff complete")
```

##### `send_velocity_command(vx: float, vy: float, vz: float, yaw_rate: float = 0.0)`

Send velocity command in NED frame.

**Parameters:**
- `vx` (float): Velocity north (m/s), range: [-5, 5]
- `vy` (float): Velocity east (m/s), range: [-5, 5]
- `vz` (float): Velocity down (m/s), range: [-2, 2]
- `yaw_rate` (float): Yaw rate (rad/s), range: [-1, 1]

**Example:**
```python
# Move forward at 2 m/s
mavlink.send_velocity_command(vx=2.0, vy=0.0, vz=0.0, yaw_rate=0.0)

# Move up at 1 m/s while turning
mavlink.send_velocity_command(vx=0.0, vy=0.0, vz=-1.0, yaw_rate=0.5)
```

##### `get_loop_timing() -> Tuple[float, float, float]`

Get control loop timing statistics.

**Returns:**
- `tuple`: (mean, std, max) loop time in milliseconds

**Example:**
```python
mean, std, max_time = mavlink.get_loop_timing()
print(f"Loop timing: {mean:.2f} ± {std:.2f}ms (max: {max_time:.2f}ms)")
```

##### `is_alive() -> bool`

Check if connection is alive.

**Returns:**
- `bool`: True if connection alive

---

## Sensors

### AirSimSensors

Interface to AirSim sensors.

#### Import
```python
from drone_gym.sensors.airsim_sensors import (
    AirSimSensors,
    LiDARData,
    CameraData,
    IMUData,
    GPSData
)
```

#### Constructor

```python
AirSimSensors(
    vehicle_name: str = "EDU650",
    airsim_host: str = "127.0.0.1",
    airsim_port: int = 41451
)
```

#### Methods

##### `connect(timeout: float = 10.0) -> bool`

Connect to AirSim.

##### `get_lidar_data() -> Optional[LiDARData]`

Get LiDAR data at 10Hz.

**Returns:**
- `LiDARData` or `None`:
  - `points` (np.ndarray): (N, 3) 3D points
  - `ranges` (np.ndarray): (360,) ranges in meters
  - `angles` (np.ndarray): (360,) angles in radians
  - `timestamp` (float): Timestamp

**Example:**
```python
sensors = AirSimSensors()
sensors.connect()

lidar_data = sensors.get_lidar_data()
if lidar_data is not None:
    print(f"LiDAR points: {len(lidar_data.points)}")
    print(f"Min range: {np.min(lidar_data.ranges):.2f}m")
    print(f"Max range: {np.max(lidar_data.ranges):.2f}m")
```

##### `get_camera_data(get_depth: bool = True, get_segmentation: bool = False) -> Optional[CameraData]`

Get camera data at 30Hz.

**Parameters:**
- `get_depth` (bool): Whether to get depth image
- `get_segmentation` (bool): Whether to get segmentation

**Returns:**
- `CameraData` or `None`:
  - `rgb` (np.ndarray): (H, W, 3) RGB image
  - `depth` (np.ndarray): (H, W) depth image
  - `segmentation` (np.ndarray): (H, W, 3) segmentation
  - `timestamp` (float): Timestamp
  - `camera_info` (dict): Camera parameters

**Example:**
```python
camera_data = sensors.get_camera_data(get_depth=True)
if camera_data is not None:
    print(f"RGB shape: {camera_data.rgb.shape}")
    print(f"Depth shape: {camera_data.depth.shape}")

    # Display or process images
    import cv2
    cv2.imshow('RGB', camera_data.rgb)
    cv2.waitKey(1)
```

##### `get_imu_data() -> Optional[IMUData]`

Get IMU data at 400Hz.

**Returns:**
- `IMUData` or `None`:
  - `linear_acceleration` (np.ndarray): (3,) m/s²
  - `angular_velocity` (np.ndarray): (3,) rad/s
  - `orientation` (np.ndarray): (4,) quaternion [w, x, y, z]
  - `timestamp` (float): Timestamp

**Example:**
```python
imu_data = sensors.get_imu_data()
if imu_data is not None:
    accel = imu_data.linear_acceleration
    gyro = imu_data.angular_velocity
    print(f"Acceleration: [{accel[0]:.2f}, {accel[1]:.2f}, {accel[2]:.2f}] m/s²")
    print(f"Gyro: [{gyro[0]:.2f}, {gyro[1]:.2f}, {gyro[2]:.2f}] rad/s")
```

##### `get_gps_data() -> Optional[GPSData]`

Get GPS data at 10Hz.

**Returns:**
- `GPSData` or `None`:
  - `latitude` (float): Latitude (degrees)
  - `longitude` (float): Longitude (degrees)
  - `altitude` (float): Altitude MSL (meters)
  - `velocity` (np.ndarray): (3,) velocity in NED (m/s)
  - `fix_type` (int): GPS fix type (0=no fix, 3=3D, 4=RTK)
  - `eph` (float): Horizontal accuracy (m)
  - `epv` (float): Vertical accuracy (m)
  - `timestamp` (float): Timestamp

**Example:**
```python
gps_data = sensors.get_gps_data()
if gps_data is not None:
    print(f"Position: {gps_data.latitude:.6f}, {gps_data.longitude:.6f}")
    print(f"Altitude: {gps_data.altitude:.2f}m")
    print(f"Accuracy: ±{gps_data.eph:.2f}m (H), ±{gps_data.epv:.2f}m (V)")
```

---

## Path Planning Algorithms

### Base PathPlanner

Abstract base class for path planners.

#### Import
```python
from drone_gym.algorithms.path_planning import (
    PathPlanner,
    PathNode,
    AStarPlanner,
    RRTStarPlanner,
    APFPlanner
)
```

#### PathNode

Data class representing a node in a path.

```python
@dataclass
class PathNode:
    position: np.ndarray  # (3,) [x, y, z]
    yaw: float = 0.0      # Heading angle (rad)
```

#### PathPlanner.plan()

```python
def plan(
    start: np.ndarray,
    goal: np.ndarray,
    obstacles: List[Tuple[np.ndarray, float]]
) -> Optional[List[PathNode]]
```

**Parameters:**
- `start` (np.ndarray): Start position (3,)
- `goal` (np.ndarray): Goal position (3,)
- `obstacles` (List): List of (center, radius) tuples

**Returns:**
- `List[PathNode]` or `None`: Path from start to goal

---

### AStarPlanner

Grid-based A* path planning algorithm.

#### Constructor

```python
AStarPlanner(
    grid_resolution: float = 0.5,
    safety_distance: float = 3.0,
    bounds: Tuple[np.ndarray, np.ndarray] = (
        np.array([-100, -100, 0]),
        np.array([100, 100, 50])
    )
)
```

**Parameters:**
- `grid_resolution` (float): Grid cell size (meters)
- `safety_distance` (float): Minimum distance from obstacles (meters)
- `bounds` (Tuple): (min_bounds, max_bounds) search space

#### Example

```python
from drone_gym.algorithms.path_planning import AStarPlanner
import numpy as np

planner = AStarPlanner(grid_resolution=0.5, safety_distance=3.0)

start = np.array([0, 0, -10])
goal = np.array([50, 50, -20])
obstacles = [
    (np.array([25, 25, -15]), 5.0),  # Obstacle at (25, 25, -15) with radius 5m
]

path = planner.plan(start, goal, obstacles)

if path is not None:
    print(f"Path found with {len(path)} nodes")
    for i, node in enumerate(path):
        print(f"  Node {i}: position={node.position}, yaw={node.yaw:.2f}")
else:
    print("No path found")
```

**Performance:**
- Computation time: <50ms (typical)
- Path optimality: >85% of optimal
- Best for: Structured environments, known obstacles

---

### RRTStarPlanner

Probabilistic RRT* path planning algorithm.

#### Constructor

```python
RRTStarPlanner(
    max_iterations: int = 1000,
    step_size: float = 2.0,
    goal_sample_rate: float = 0.1,
    search_radius: float = 5.0,
    safety_distance: float = 3.0
)
```

**Parameters:**
- `max_iterations` (int): Maximum iterations
- `step_size` (float): Maximum step size (meters)
- `goal_sample_rate` (float): Probability of sampling goal (0-1)
- `search_radius` (float): Radius for finding nearby nodes (meters)
- `safety_distance` (float): Minimum distance from obstacles (meters)

#### Example

```python
from drone_gym.algorithms.path_planning import RRTStarPlanner

planner = RRTStarPlanner(
    max_iterations=1000,
    step_size=2.0,
    search_radius=5.0
)

path = planner.plan(start, goal, obstacles)
```

**Performance:**
- Computation time: <100ms
- Path optimality: Improves with iterations
- Best for: Complex environments, many obstacles

---

### APFPlanner

Artificial Potential Field reactive planner.

#### Constructor

```python
APFPlanner(
    k_att: float = 1.0,
    k_rep: float = 50.0,
    d0: float = 5.0,
    max_iterations: int = 500,
    step_size: float = 0.5,
    goal_threshold: float = 1.0,
    safety_distance: float = 3.0
)
```

**Parameters:**
- `k_att` (float): Attractive potential gain
- `k_rep` (float): Repulsive potential gain
- `d0` (float): Influence distance of obstacles (meters)
- `max_iterations` (int): Maximum iterations
- `step_size` (float): Step size for gradient descent (meters)
- `goal_threshold` (float): Distance to goal for termination (meters)
- `safety_distance` (float): Minimum distance from obstacles (meters)

#### Example

```python
from drone_gym.algorithms.path_planning import APFPlanner

planner = APFPlanner(
    k_att=1.0,
    k_rep=50.0,
    d0=5.0
)

path = planner.plan(start, goal, obstacles)
```

**Performance:**
- Computation time: <20ms (very fast)
- Path optimality: Variable (can get stuck in local minima)
- Best for: Dynamic environments, real-time replanning

---

## Utilities

### Reward Function Components

The reward function consists of 7 components:

```python
def _calculate_reward(self, position, velocity, action, state):
    reward = 0.0

    # 1. Distance to goal: +1.0 per meter closer
    distance_improvement = last_distance - current_distance
    reward += 1.0 * distance_improvement

    # 2. Mission completion: +50
    if distance < 2.0:
        reward += 50.0

    # 3. Collision penalty: -100
    if collision_detected:
        reward -= 100.0

    # 4. Energy efficiency: +0.1 * efficiency
    efficiency = 1.0 - (actual_usage / estimated_usage)
    reward += 0.1 * efficiency

    # 5. Smooth flight: +0.05 * smoothness
    jerk = np.linalg.norm(velocity_change) / dt
    smoothness = 1.0 - min(jerk / 10.0, 1.0)
    reward += 0.05 * smoothness

    # 6. No-fly zone violation: -50
    if in_no_fly_zone:
        reward -= 50.0

    # 7. Time penalty: -0.01 per second
    reward -= 0.01

    return reward
```

---

## Configuration

### ArduPilot Parameters

Key parameters in `configs/ardupilot/params.parm`:

```
# Control Loop
SCHED_LOOP_RATE=50           # 50Hz (20ms cycle)

# Motor Configuration
MOT_THST_HOVER=0.35          # 35% hover throttle for 3.4kg

# Velocity Control
PSC_VELXY_P=1.0              # Horizontal velocity P gain
PSC_VELZ_P=5.0               # Vertical velocity P gain

# Navigation
WPNAV_SPEED=500              # 5 m/s horizontal speed
WPNAV_SPEED_UP=250           # 2.5 m/s climb rate
WPNAV_RADIUS=200             # 2m waypoint radius

# Safety
FENCE_ENABLE=1               # Geofence enabled
FENCE_ALT_MAX=120            # 120m max altitude
AVOID_ENABLE=7               # All avoidance features
```

### AirSim Settings

Key settings in `configs/airsim/settings.json`:

```json
{
  "Vehicles": {
    "EDU650": {
      "VehicleType": "SimpleFlight",
      "Parameters": {
        "VehicleMass": 3.4,
        "RotorCount": 4,
        "RotorConfiguration": "X"
      },
      "Sensors": {
        "Imu": {"UpdateFrequency": 400.0},
        "Gps": {"UpdateFrequency": 10.0},
        "LidarFront": {
          "NumberOfChannels": 360,
          "Range": 16.0,
          "RotationsPerSecond": 10
        }
      }
    }
  }
}
```

---

## Complete Example: Custom Training Loop

```python
import gym
import numpy as np
from drone_gym import DroneNavEnv
from drone_gym.controllers.mavlink_interface import MAVLinkInterface
from drone_gym.sensors.airsim_sensors import AirSimSensors
from drone_gym.algorithms.path_planning import AStarPlanner

# Create environment
env = gym.make('DroneNav-v0')

# Connect
if not env.connect(timeout=30.0):
    print("Connection failed")
    exit(1)

# Arm and takeoff
env.mavlink.set_mode('GUIDED')
env.mavlink.arm()
env.mavlink.takeoff(altitude=10.0)

# Create planner
planner = AStarPlanner()

# Training loop
for episode in range(100):
    obs = env.reset()
    episode_reward = 0

    # Plan path
    state = env.mavlink.get_state()
    start = state['position']
    goal = env.target_position

    path = planner.plan(start, goal, env.obstacles)

    if path is None:
        print(f"Episode {episode}: No path found")
        continue

    # Follow path
    for step in range(1000):
        # Simple path following controller
        current_pos = env.mavlink.get_state()['position']
        target_pos = path[min(step // 10, len(path) - 1)].position

        # Proportional control
        error = target_pos - current_pos
        action = np.clip(error * 0.5, -5.0, 5.0)
        action = np.append(action, 0.0)  # Add yaw_rate

        # Step
        obs, reward, done, info = env.step(action)
        episode_reward += reward

        if done:
            break

    print(f"Episode {episode}: Reward={episode_reward:.2f}, "
          f"Success={info['mission_completed']}")

# Cleanup
env.mavlink.set_mode('LAND')
env.close()
```

---

## Error Handling

### Common Exceptions

```python
try:
    env = gym.make('DroneNav-v0')
    env.connect(timeout=10.0)
except ImportError:
    print("Error: AirSim not installed")
except ConnectionError:
    print("Error: Cannot connect to SITL or AirSim")
except TimeoutError:
    print("Error: Connection timeout")
except Exception as e:
    print(f"Error: {e}")
```

### Checking Connection Status

```python
# Check MAVLink connection
if not env.mavlink.is_alive():
    print("MAVLink connection lost")
    env.mavlink.connect()

# Check AirSim connection
if not env.sensors.connected:
    print("AirSim connection lost")
    env.sensors.connect()
```

---

## Performance Tips

### 1. Optimize Control Loop

```python
# Use faster simulation speed for training
# In start_sitl.sh:
./scripts/start_sitl.sh --speedup 10

# Reduce sensor update rates if not needed
sensors.lidar_rate = 5.0  # Reduce from 10Hz to 5Hz
```

### 2. Parallel Environments

```python
from stable_baselines3.common.vec_env import SubprocVecEnv

def make_env():
    return gym.make('DroneNav-v0')

# Create 8 parallel environments
envs = SubprocVecEnv([make_env for _ in range(8)])
```

### 3. Reduce Observation Space

```python
# If you don't need camera features, modify observation
class SimplifiedDroneEnv(BaseDroneEnv):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # Redefine observation space without camera
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(376,),  # 360 + 16 (no camera)
            dtype=np.float32
        )
```

---

## Version Information

- **API Version**: 0.1.0
- **Python**: 3.8+
- **Gym**: 0.26.x
- **Stable-Baselines3**: 2.0+
- **PyMAVLink**: 2.4.37+
- **AirSim**: 1.8.1+

---

## Support

For API questions:
- Check examples in `scripts/`
- See [Operation Manual](OPERATION_MANUAL.md)
- Create an issue in the repository

---

**End of API Reference**
