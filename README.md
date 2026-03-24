# prometheus DiffDrive Robot

<https://github.com/bruk/prometheus/assets/>

## About

This repository contains a Gazebo and Isaac Sim simulation for a differential drive robot, equipped with an IMU, a depth camera, stereo camera and a 2D LiDAR. The primary contriution of this project is to support multiple ROS and Gazebo distros. Currently, the project supports the following versions -

1. [ROS2 Humble/Jazzy with Gazebo Harmonic](#humble--harmonic-ubuntu-2204)

Each of the following sections describes depedencies, build and run instructions for each of the above combinations

## ROS2 Humble/Jazzy with Gazebo Harmonic

### Dependencies

- [AWS RoboMaker Small House World ROS package](https://github.com/aws-robotics/aws-robomaker-small-house-world)
```bash
sudo apt-get install ros-humble-ros-gzharmonic
```

```bash
# From the root directory of the workspace. This will install everything mentioned in package.xml
rosdep install --from-paths src/prometheus --ignore-src -r -y
```

### Build

```bash
colcon build --packages-select prometheus
```

### Run

To launch the robot in Gazebo,

```bash
ros2 launch prometheus gz.launch.py
```

To view in rviz,

```bash
ros2 launch prometheus rviz.launch.py
```

### Configuration

The launch file accepts multiple launch arguments,

```bash
ros2 launch prometheus gz.launch.py \
 arm_enabled:=True \
 camera_enabled:=True \
 stereo_camera_enabled:=False \
 two_d_lidar_enabled:=True \
 position_x:=0.0 \
 position_y:=0.0  \
 orientation_yaw:=0.0 \
 odometry_source:=world \
 world_file:=small_warehouse.sdf
```

### Mapping with SLAM Toolbox

SLAM Toolbox is an open-source package designed to map the environment using laser scans and odometry, generating a map for autonomous navigation.

NOTE: The command to run mapping is common between all versions of gazebo.

To start mapping:

```bash
ros2 launch prometheus mapping.launch.py
```

Use the teleop twist keyboard to control the robot and map the area:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/prometheus/cmd_vel
```

To save the map:

```bash
cd src/prometheus/config
ros2 run nav2_map_server map_saver_cli -f map
```

### Using Nav2 with prometheus

Nav2 is an open-source navigation package that enables a robot to navigate through an environment easily. It takes laser scan and odometry data, along with the map of the environment, as inputs.

NOTE: The command to run navigation is common between all versions of gazebo and Isaac sim.

To run Nav2 on prometheus:

```bash
ros2 launch prometheus nav2.launch.py
```

### Arm Control Interface

The SO-100 5-DOF arm is controlled via a unified ROS2 interface using [Pink](https://github.com/stephane-caron/pink) (Pinocchio-based IK). The `pink_ik_node` exposes action servers and services for arm movement, state queries, and gripper control.

#### Arm Controllers

The arm uses `ros2_control` with the following controllers (loaded automatically by the launch file):

| Controller | Type | Purpose |
|---|---|---|
| `joint_state_broadcaster` | JointStateBroadcaster | Publishes joint states |
| `joint_trajectory_controller` | JointTrajectoryController | Position control for 5 arm joints |
| `gripper_controller` | GripperActionController | Gripper open/close |

Arm joints: `Shoulder_Rotation`, `Shoulder_Pitch`, `Elbow`, `Wrist_Pitch`, `Wrist_Roll`

#### Dependencies

```bash
pip install pin-pink quadprog "numpy<2"
```

> **Note:** `numpy<2` is required because the ROS Jazzy system Pinocchio is compiled against NumPy 1.x. The `pin-pink` package pulls NumPy 2.x by default, which causes a crash.

#### Running

```bash
# Terminal 1: Launch simulation
ros2 launch prometheus gz.launch.py

# Terminal 2: Launch arm control node (loads controllers automatically)
ros2 launch prometheus pink_ik.launch.py
```

#### ROS2 Interface

##### Action: `arm/move_to_pose` (`prometheus/action/MoveArm`)

Unified arm movement with 5 modes:

| Mode | Description | Required Fields |
|---|---|---|
| `cartesian` | Move EE to absolute Cartesian pose | `target_pose` |
| `relative` | Move EE by offset from current position | `offset` (dx, dy, dz) |
| `named` | Move to a named joint configuration | `pose_name` (`home`, `ready`, `tucked`) |
| `home` | Return to home position | none |
| `joints` | Move to direct joint angles | `joint_positions` (5 floats, radians) |

Feedback provides `progress` (0-1), `position_error` (meters), and `phase` (`solving`/`executing`).

Example:

```bash
# Cartesian goal
ros2 action send_goal /arm/move_to_pose prometheus/action/MoveArm \
  "{mode: 'cartesian', target_pose: {header: {frame_id: 'base_footprint'}, pose: {position: {x: 0.3, y: 0.0, z: 0.3}, orientation: {w: 1.0}}}}"

# Relative move (10cm down)
ros2 action send_goal /arm/move_to_pose prometheus/action/MoveArm \
  "{mode: 'relative', offset: {z: -0.1}}"

# Named pose
ros2 action send_goal /arm/move_to_pose prometheus/action/MoveArm \
  "{mode: 'named', pose_name: 'home'}"

# Home
ros2 action send_goal /arm/move_to_pose prometheus/action/MoveArm "{mode: 'home'}"
```

##### Gripper: `gripper_controller/gripper_cmd` (`control_msgs/action/GripperCommand`)

Standard ROS2 gripper action. Position 0.0 (closed) to 1.57 (open).

```bash
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand \
  "{command: {position: 0.0, max_effort: 100.0}}"
```

##### Topics

| Topic | Type | Description |
|---|---|---|
| `pink_ik/current_ee_pose` | `geometry_msgs/PoseStamped` | Current FK end-effector pose at 10 Hz |
| `pink_ik/status` | `std_msgs/String` | Solver status: `idle`, `solving`, `executing`, `reached`, `failed` |
| `prometheus/joint_states` | `sensor_msgs/JointState` | All joint positions (arm + gripper) |

#### Configuration

IK parameters are in `config/pink_ik_params.yaml`:

- `position_cost` / `orientation_cost` — task weights (position prioritized for 5-DOF arm)
- `max_iterations` / `dt` — differential IK loop settings
- `position_threshold` — convergence criterion (meters)
- `trajectory_duration` / `num_waypoints` — trajectory execution timing
- `named_poses` — dict of named joint configurations (home, ready, tucked)

### Simulation and Visualization

1. Gz Sim (small_house World):
 ![](img/gz1.png)
 ![](img/gz2.png)

2. Rviz (small_house World):
 ![](img/rviz1.png)
 ![](img/rviz2.png)

## Acknowledgment

this project is heavily based on the follwing repos
- [BCR_BOT](https://github.com/blackcoffeerobotics/bcr_bot)
- [SO-ARM-100](https://github.com/brukg/so-100-arm)