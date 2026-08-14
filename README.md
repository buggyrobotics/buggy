<p align="center">
  <img src="https://img.shields.io/badge/ROS%202-Humble%20|%20Iron-blue?logo=ros&logoColor=white" alt="ROS 2">
  <img src="https://img.shields.io/badge/Gazebo-Fortress%20|%20Harmonic-orange?logo=gazebo&logoColor=white" alt="Gazebo">
  <img src="https://img.shields.io/badge/License-Apache%202.0-green?logo=apache&logoColor=white" alt="License">
  <img src="https://img.shields.io/badge/Build-ament__cmake-lightgrey?logo=cmake&logoColor=white" alt="Build">
</p>

<h1 align="center"> Buggy — Nvis 3302ARD Autonomous Rover</h1>

<p align="center">
  <strong>A complete ROS 2 package for the Nvis 3302ARD differential-drive rover — from URDF description and Gazebo simulation to real-hardware SLAM mapping and autonomous Nav2 navigation.</strong>
</p>

---

##  Table of Contents

- [Overview](#-overview)
- [Hardware Platform](#-hardware-platform)
  - [Chassis & Mechanical Assembly](#chassis--mechanical-assembly)
  - [Drive System](#drive-system)
  - [Caster Wheel](#caster-wheel)
  - [LiDAR Sensor](#lidar-sensor)
  - [Kinematic Model](#kinematic-model)
- [Software Architecture](#-software-architecture)
  - [TF Frame Tree](#tf-frame-tree)
  - [ROS 2 Node Graph](#ros-2-node-graph)
- [Repository Structure](#-repository-structure)
- [Prerequisites](#-prerequisites)
- [Installation](#-installation)
- [Usage](#-usage)
  - [Simulation (Gazebo)](#1-simulation-gazebo)
  - [Real Hardware Bringup](#2-real-hardware-bringup)
  - [SLAM Mapping](#3-slam-mapping)
  - [Autonomous Navigation (Nav2)](#4-autonomous-navigation-nav2)
- [Configuration Reference](#-configuration-reference)
- [Troubleshooting](#-troubleshooting)
- [Contributing](#-contributing)
- [License](#-license)

---

## Overview

The **Buggy** package provides everything needed to operate the Nvis 3302ARD rover — a compact, differential-drive robot designed for indoor crowd navigation and autonomous exploration.

| Capability | Stack | Details |
|---|---|---|
| **Robot Description** | URDF / Xacro | Parameterised model with collision, inertial, and visual properties |
| **Simulation** | Gazebo (gz-sim) | Full physics sim via `gz_ros2_control` plugin |
| **Motor Control** | `ros2_control` | `diff_drive_controller` with velocity command interface |
| **Odometry** | Wheel encoders | Closed-loop odometry via joint state feedback |
| **Perception** | 2D LiDAR | Budget 8 m / 5 Hz GPU-accelerated laser scanner |
| **Mapping** | SLAM Toolbox | Online asynchronous SLAM with loop closure |
| **Localization** | Nav2 AMCL | Adaptive Monte Carlo particle filter |
| **Path Planning** | NavFn (A*) | Global planner with path smoothing |
| **Path Following** | DWB Local Planner | Dynamic Window approach with configurable critics |
| **Recovery Behaviors** | Nav2 Behaviors | Spin, backup, drive-on-heading, wait |
| **Teleoperation** | Any `/cmd_vel` source | Compatible with `teleop_twist_keyboard`, joystick, etc. |

---

##  Hardware Platform

### Chassis & Mechanical Assembly

The Nvis 3302ARD uses a rectangular box chassis with the following measured dimensions:

```
                    300 mm
        ┌─────────────────────────┐
        │                         │
        │      ┌───────────┐      │
        │      │           │      │
 300 mm │      │  CHASSIS  │      │  Height: 200 mm
        │      │  (1.0 kg) │      │
        │      │           │      │
        │      └───────────┘      │
        │                         │
        └─────────────────────────┘
```

| Property | Value | Source |
|---|---|---|
| Chassis dimensions (L × W × H) | 300 × 300 × 200 mm | `nvis_3302ard.urdf.xacro` |
| Total robot mass | 1.0 kg | `nvis_3302ard.urdf.xacro` |
| Chassis mass (computed) | 0.75 kg | `total_mass - 2×wheel_mass - caster_mass` |
| Ground clearance | 5 mm | `chassis_ground_clearance` property |
| Collision radius | 140 mm | `robot_collision_radius` property |
| Costmap footprint | 350 × 350 mm square | `nav2_params.yaml` footprint polygon |
| Footprint padding | 30 mm | `nav2_params.yaml` |

### Drive System

The rover uses a **two-wheel differential drive** configuration with independently velocity-controlled wheels.

| Property | Value | Source |
|---|---|---|
| Drive type | Differential drive | `diff_drive_controller` |
| Wheel radius | 32.5 mm | `wheel_radius` xacro property |
| Wheel width | 26 mm | `wheel_width` xacro property |
| Wheel separation (axle-to-axle) | 180 mm | `wheel_separation` xacro / controller YAML |
| Wheel mass (each) | 50 g | `wheel_mass` xacro property |
| Wheel geometry | Cylinder | URDF visual/collision |
| Wheel friction (µ) | 1.0 | Gazebo `<mu1>` / `<mu2>` |
| Joint type | Continuous (infinite rotation) | `left_wheel_joint`, `right_wheel_joint` |
| Command interface | Velocity (rad/s) | `ros2_control` block |
| Velocity command range | ±10 rad/s | `ros2_control` min/max params |
| State interfaces | Position + Velocity | `ros2_control` block |
| Joint damping | 0.05 N·m·s/rad | `<dynamics>` tag |
| Hardware plugin (sim) | `gz_ros2_control/GazeboSimSystem` | `ros2_control.urdf.xacro` |

#### Velocity & Acceleration Limits

| Parameter | Linear (x) | Angular (z) | Source |
|---|---|---|---|
| Max velocity | 1.0 m/s | 2.0 rad/s | `diff_drive_controller.yaml` |
| Min velocity | −0.3 m/s | −2.0 rad/s | `diff_drive_controller.yaml` |
| Max acceleration | 0.8 m/s² | 3.0 rad/s² | `diff_drive_controller.yaml` |
| Max jerk | 5.0 m/s³ | 10.0 rad/s³ | `diff_drive_controller.yaml` |
| Nav2 max velocity | 0.3 m/s | 1.5 rad/s | `nav2_params.yaml` (FollowPath) |

### Caster Wheel

A passive spherical caster at the rear provides stability for the three-point contact configuration.

| Property | Value | Source |
|---|---|---|
| Caster type | Spherical (ball caster) | URDF `<sphere>` geometry |
| Caster radius | 12 mm | `caster_radius` xacro property |
| Caster mass | 150 g | `caster_mass` xacro property |
| Caster X-offset from base | −70 mm (rear) | `caster_offset_x` xacro property |
| Caster joint | Fixed | `caster_wheel_joint` |
| Caster friction (µ) | 0.001 | Gazebo `<mu1>` / `<mu2>` (near-frictionless) |

> **Design note:** The caster friction is intentionally set near-zero (0.001) so it does not interfere with the differential-drive steering — it slides freely while supporting the chassis.

### LiDAR Sensor

The rover is equipped with a **budget-class 2D LiDAR** mounted on top of the chassis. The codebase describes it as a _"deliberately conservative LiDAR spec: budget 8m/5Hz/~1° sensor, not a premium unit."_

| Property | Default Value | Configurable | Source |
|---|---|---|---|
| Sensor type | 2D GPU LiDAR | — | Gazebo `gpu_lidar` plugin |
| Min range | 0.12 m |  `range_min` | `lidar.urdf.xacro` |
| Max range | 8.0 m |  `range_max` | `lidar.urdf.xacro` |
| Update rate | 5.0 Hz |  `update_rate` | `lidar.urdf.xacro` |
| Horizontal FOV | 360° (6.283 rad) |  `horizontal_fov` | `lidar.urdf.xacro` default |
| Angular resolution | ~1° (360 samples) |  `samples` | `lidar.urdf.xacro` |
| Range resolution | 0.01 m | — | Gazebo `<resolution>` |
| Noise model | Gaussian | — | `lidar.urdf.xacro` |
| Noise std. deviation | 0.02 m |  `range_noise_stddev` | `lidar.urdf.xacro` |
| Vertical FOV | 0° (single ray) | — | Explicit single-ray config |
| Mount shape | Cylinder (puck) | — | URDF visual |
| Mount radius | 20 mm |  `mount_radius` | `lidar.urdf.xacro` |
| Mount height | 20 mm |  `mount_height` | `lidar.urdf.xacro` |
| Sensor mass | 20 g | — | URDF inertial |
| Mount position | Top of chassis + 30 mm standoff | — | `lidar_standoff` property |
| ROS topic | `/scan` | — | `gz_ros2_bridge` / launch config |
| Frame ID | `lidar_link` | — | `<gz_frame_id>` |

> **Real hardware note:** In `bringup_launch.py`, the LiDAR driver is a placeholder — replace `your_lidar_driver_pkg` / `your_lidar_driver_node` with your actual driver (e.g. `rplidar_ros`, `ldlidar_stl_ros2`, `sick_scan_xd`). It must publish `sensor_msgs/LaserScan` on `/scan`.

#### Override in Main Robot Model

When instantiated in the main robot description (`nvis_3302ard.urdf.xacro`), the LiDAR FOV is narrowed:

| Override | Value | Reason |
|---|---|---|
| `horizontal_fov` | 180° (π rad) | Front-facing only configuration |
| `samples` | 180 | ~1° angular resolution maintained |

### Kinematic Model

```
              ← 180 mm →
         ┌──┐           ┌──┐
         │LW│           │RW│    Wheel ⌀ 65 mm
         └──┘           └──┘    Wheel width 26 mm
           ╲      ↑      ╱
            ╲     │     ╱
             ╲    │    ╱
              ╲   │   ╱
               ╲  │  ╱
       ─────────[BASE]─────────  base_link (at axle height)
                  │
                  │  70 mm
                  │
                 (●)             Caster ⌀ 24 mm
                                 µ ≈ 0 (free-sliding)

       ▲ Forward (+X)
```

**Frame hierarchy:** `base_footprint` → `base_link` (offset Z = wheel_radius = 32.5 mm) → wheels, caster, lidar

---

##  Software Architecture

### TF Frame Tree

```
map
 └── odom                          (published by diff_drive_controller)
      └── base_footprint           (ground-plane projection)
           └── base_link           (chassis centre at axle height)
                ├── left_wheel     (continuous joint — revolves)
                ├── right_wheel    (continuous joint — revolves)
                ├── caster_wheel   (fixed joint)
                └── lidar_link     (fixed joint — sensor frame)
```

### ROS 2 Node Graph

```
┌─────────────────────┐     ┌──────────────────────┐
│  robot_state_pub    │────▶│  /robot_description   │
└─────────────────────┘     │  /tf, /tf_static      │
                            └──────────────────────┘
┌─────────────────────┐     ┌──────────────────────┐
│  controller_manager │────▶│  /joint_states        │
│  (ros2_control)     │     │  /diff_drive_ctrl/odom│
└─────────────────────┘     └──────────────────────┘
        ▲ cmd_vel                     │
        │                             ▼
┌───────┴─────────┐          ┌──────────────────┐
│  Nav2 / teleop  │          │  slam_toolbox    │◀── /scan
└─────────────────┘          └──────────────────┘
                                      │
                                      ▼ /map
                             ┌──────────────────┐
                             │  Nav2 Stack       │
                             │  (AMCL, planner,  │
                             │   controller, BT) │
                             └──────────────────┘
```

Key topic relay: Nav2 publishes on `/cmd_vel` → a `topic_tools/relay` node forwards to `/diff_drive_controller/cmd_vel_unstamped`.

---

##  Repository Structure

```
buggy/
├── config/
│   ├── diff_drive_controller.yaml    # ros2_control controller params (100 Hz update)
│   ├── nav2_params.yaml              # Full Nav2 stack config (AMCL, DWB, costmaps)
│   └── slam_toolbox_params.yaml      # SLAM Toolbox online-async with loop closure
│
├── launch/
│   ├── bringup_launch.py             # Real-hardware bringup (controller_manager + LiDAR)
│   ├── slam.launch.py                # SLAM mapping (async_slam_toolbox_node)
│   └── nav2.launch.py                # Nav2 navigation with cmd_vel relay
│
├── urdf/
│   ├── nvis_3302ard.urdf.xacro       # Main robot model (chassis, wheels, caster, LiDAR, Gazebo)
│   ├── nvis_3302ard.ros2_control.urdf.xacro  # Hardware interface definition (GazeboSimSystem)
│   └── lidar.urdf.xacro              # Reusable parameterised LiDAR macro
│
├── CMakeLists.txt                    # ament_cmake build (installs launch/config/urdf/maps/worlds)
├── package.xml                       # ROS 2 package manifest (format 3)
├── LICENSE                           # Apache License 2.0
└── README.md
```

---

##  Prerequisites

| Requirement | Version |
|---|---|
| Ubuntu | 22.04 (Jammy) or 24.04 (Noble) |
| ROS 2 | Humble / Iron / Jazzy |
| Gazebo | Fortress or Harmonic (for simulation) |
| `ros2_control` | Matching your ROS 2 distro |
| `slam_toolbox` | Matching your ROS 2 distro |
| Navigation2 | Matching your ROS 2 distro |

### Install All Dependencies

```bash
sudo apt update && sudo apt install -y \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-ros-gz-sim \
  ros-${ROS_DISTRO}-ros-gz-bridge \
  ros-${ROS_DISTRO}-gz-ros2-control \
  ros-${ROS_DISTRO}-controller-manager \
  ros-${ROS_DISTRO}-diff-drive-controller \
  ros-${ROS_DISTRO}-joint-state-broadcaster \
  ros-${ROS_DISTRO}-slam-toolbox \
  ros-${ROS_DISTRO}-nav2-bringup \
  ros-${ROS_DISTRO}-topic-tools \
  ros-${ROS_DISTRO}-teleop-twist-keyboard
```

---

##  Installation

```bash
# 1. Create or use an existing ROS 2 workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# 2. Clone
git clone https://github.com/buggyrobotics/buggy.git

# 3. Resolve remaining rosdep dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
colcon build --symlink-install --packages-select buggy

# 5. Source
source install/setup.bash
```

---

##  Usage

> **Reminder:** Run `source ~/ros2_ws/install/setup.bash` in every new terminal.

### 1. Simulation (Gazebo)

```bash
# Launch robot + Gazebo + controllers
ros2 launch buggy bringup_launch.py

# In a second terminal — teleoperate
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The launch file starts:
- `robot_state_publisher` with the URDF from Xacro
- `ros2_control_node` (controller manager)
- `joint_state_broadcaster` (spawned after 3 s delay)
- `diff_drive_controller` (spawned after 6 s delay)
- LiDAR driver node

### 2. Real Hardware Bringup

**Before launching**, edit [`bringup_launch.py`](launch/bringup_launch.py) and replace the placeholder LiDAR driver:

```python
# Replace these two lines with your actual LiDAR driver
lidar_driver = Node(
    package='your_lidar_driver_pkg',       # ← e.g. 'rplidar_ros'
    executable='your_lidar_driver_node',   # ← e.g. 'rplidar_node'
    ...
)
```

Then launch:

```bash
ros2 launch buggy bringup_launch.py
```

### 3. SLAM Mapping

Build a map while teleoperating the rover:

```bash
# Terminal 1 — robot bringup
ros2 launch buggy bringup_launch.py

# Terminal 2 — SLAM
ros2 launch buggy slam.launch.py

# Terminal 3 — drive around
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Save the finished map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

SLAM Toolbox is configured for **online asynchronous** mode with:
- Map resolution: 0.05 m/cell
- Max laser range: 8.0 m
- Loop closure: **enabled** (search radius 3.0 m, chain size ≥ 10)
- Scan matching: enabled with barycenter

### 4. Autonomous Navigation (Nav2)

Requires a previously saved map.

```bash
# Terminal 1 — robot bringup
ros2 launch buggy bringup_launch.py

# Terminal 2 — Nav2 stack
ros2 launch buggy nav2.launch.py
```

Open **RViz 2** and:
1. Set initial pose with the **2D Pose Estimate** tool
2. Send goals with the **Nav2 Goal** tool

The Nav2 stack includes:

| Component | Plugin | Key Setting |
|---|---|---|
| **Localization** | AMCL (likelihood field) | 1000–3000 particles |
| **Global Planner** | NavFn (A*) | Tolerance: 0.5 m |
| **Local Planner** | DWB (Dynamic Window) | 7 critics: RotateToGoal, Oscillation, BaseObstacle, GoalAlign, PathAlign, PathDist, GoalDist |
| **Path Smoother** | SimpleSmoother | Refinement enabled |
| **Velocity Smoother** | Nav2 VelocitySmoother | Open-loop, 20 Hz |
| **Recovery** | Spin, Backup, DriveOnHeading, Wait | — |
| **Navigator** | BT Navigator | NavigateToPose + NavigateThroughPoses |
| **Waypoint Following** | WaypointFollower | 200 ms pause at each waypoint |

---

## ⚙ Configuration Reference

### Diff Drive Controller — `config/diff_drive_controller.yaml`

| Parameter | Value | Description |
|---|---|---|
| `update_rate` | 100 Hz | Controller manager loop rate |
| `publish_rate` | 50 Hz | Odometry publishing rate |
| `wheel_separation` | 0.180 m | Distance between wheel centres |
| `wheel_radius` | 0.0325 m | Driven wheel radius |
| `open_loop` | `false` | Uses encoder feedback (closed-loop) |
| `enable_odom_tf` | `true` | Publishes `odom → base_footprint` TF |
| `cmd_vel_timeout` | 0.5 s | Stops motors if no command received |
| `use_stamped_vel` | `false` | Accepts `geometry_msgs/Twist` |

### SLAM Toolbox — `config/slam_toolbox_params.yaml`

| Parameter | Value | Description |
|---|---|---|
| `resolution` | 0.05 m | Map grid resolution |
| `max_laser_range` | 8.0 m | Maximum usable laser range |
| `minimum_travel_distance` | 0.1 m | Min movement before processing |
| `do_loop_closing` | `true` | Enables loop closure |
| `loop_search_maximum_distance` | 3.0 m | Loop closure search radius |
| `correlation_search_space_dimension` | 0.5 m | Scan match search window |

### Nav2 Costmaps — `config/nav2_params.yaml`

| Parameter | Local | Global |
|---|---|---|
| Window size | 5 × 5 m (rolling) | Full map |
| Resolution | 0.05 m | 0.05 m |
| Footprint | 350 × 350 mm | 350 × 350 mm |
| Inflation radius | 0.14 m | 0.14 m |
| Cost scaling factor | 10.0 | 10.0 |
| Obstacle max range | 2.5 m | 2.5 m |
| Raytrace max range | 3.0 m | 3.0 m |
| Layers | Obstacle + Inflation | Static + Obstacle + Inflation |

---

##  Troubleshooting

| Issue | Cause | Fix |
|---|---|---|
| Robot spins in place | `wheel_separation` mismatch between URDF and controller YAML | Ensure both use `0.180` |
| LiDAR self-detection | Sensor too close to chassis | Increase `lidar_standoff` (default 30 mm) |
| Odometry drift | Open-loop mode or wrong wheel radius | Verify `open_loop: false` and calibrate `wheel_radius` |
| Nav2 won't start | Missing map file | Ensure `maps/my_map_2.yaml` exists or pass `map:=` argument |
| Controllers not spawning | Timing issue | Increase `TimerAction` delays in `bringup_launch.py` (default 3 s / 6 s) |
| `/cmd_vel` not reaching motors | Topic mismatch | Nav2 relay forwards `/cmd_vel` → `/diff_drive_controller/cmd_vel_unstamped` |

---

##  Contributing

Contributions are welcome! Here's how to get started:

1. **Fork** this repository
2. Create a feature branch: `git checkout -b feature/my-feature`
3. Commit your changes: `git commit -m "feat: add my feature"`
4. Push to your fork: `git push origin feature/my-feature`
5. Open a **Pull Request**

Please follow [Conventional Commits](https://www.conventionalcommits.org/) for commit messages.

---

##  License

This project is licensed under the **Apache License 2.0** — see the [LICENSE](LICENSE) file for details.

---

<p align="center">
  Made with ❤️ by the <a href="https://github.com/buggyrobotics">Buggy Robotics</a> team
</p>
