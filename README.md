# RouteServerImplementation

A ROS2 package (`titan_nav`) that implements the **Nav2 Route Server** for autonomous mobile robot navigation in a warehouse simulation environment.

---

## Overview

This package provides a complete warehouse robotics navigation stack featuring:

- **Nav2 Route Server** — graph-based route planning using a GeoJSON waypoint graph
- **Warehouse simulation** — Gazebo world with AWS RoboMaker warehouse assets (shelves, walls, equipment)
- **Titan robot** — differential-drive mobile robot with LiDAR, Kinect RGB-D, and stereo cameras
- **AMCL localisation** — Adaptive Monte Carlo Localisation on a pre-built occupancy map
- **Custom nodes** — planner metric reporter, velocity dynamics filter, and joystick teleop bridge

---

## Package Structure

```
titan_nav/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── route.yaml                    # Nav2 / Route Server parameters
│   ├── nav2_param.yaml               # AMCL and controller parameters
│   ├── warehouse_graph_fixed.geojson # Route graph (7 nodes, 12 edges)
│   ├── coverage.yaml
│   └── joystick.yaml
├── launch/
│   ├── bringup.launch.py             # Gazebo + robot state publisher
│   ├── route.launch.py               # Nav2 stack + Route Server
│   └── rsp.launch.py                 # Robot State Publisher + bridges
├── maps/
│   ├── warehouse_map.pgm             # Occupancy grid image
│   ├── warehouse_map.yaml            # Map metadata
│   └── warehouse_graph_fixed.geojson
├── models/                           # 14 AWS RoboMaker warehouse models
├── src/
│   ├── planner_metric_node.cpp       # Route metric reporter
│   ├── controller_dynamics_node.cpp  # Velocity low-pass filter
│   └── joy_node.cpp                  # Joystick → cmd_vel bridge
├── urdf/
│   └── titan.urdf                    # Robot description
└── worlds/
    └── small_warehouse.world         # Gazebo simulation world
```

---

## Prerequisites

| Requirement | Version |
|-------------|---------|
| ROS 2       | Humble or later |
| Nav2        | Matching ROS 2 version |
| Gazebo (Ignition) | Fortress or later |
| `nav2_route` | Available in Nav2 nightly / Iron+ |

Install ROS 2 Nav2 and Gazebo:

```bash
sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup
sudo apt install ros-$ROS_DISTRO-ros-gz
```

---

## Build

```bash
# Clone into a colcon workspace
mkdir -p ~/titan_ws/src
cd ~/titan_ws/src
git clone https://github.com/harunkurtdev/RouteServerImplementation.git

# Install dependencies
cd ~/titan_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

---

## Run

### 1. Launch the Gazebo simulation

```bash
ros2 launch titan_nav bringup.launch.py
```

This starts Gazebo with the `small_warehouse.world` and spawns the Titan robot.

### 2. Launch the Nav2 + Route Server stack

In a new terminal (with the workspace sourced):

```bash
ros2 launch titan_nav route.launch.py
```

This starts:
- **Map Server** (warehouse occupancy map)
- **AMCL** localisation
- **Route Server** (graph-based routing from `warehouse_graph_fixed.geojson`)
- **BT Navigator** (behaviour-tree-based navigation)
- **Controller Server**
- **Velocity Smoother**
- **Global / Local Costmaps**
- **Lifecycle Manager**

### 3. (Optional) Joystick teleoperation

```bash
ros2 run titan_nav joy_node
```

Hold **LB (button 4)** as a deadman switch and use the left stick (linear) and right stick (angular) to drive the robot.

### 4. (Optional) Route metric monitoring

```bash
ros2 run titan_nav planner_metric_node
```

Logs path length, distance traveled, distance remaining, and ETA at 1 Hz.

---

## Warehouse Route Graph

The navigation graph (`config/warehouse_graph_fixed.geojson`) defines **7 waypoints** connected by **12 directed edges**:

| Node | X (m) | Y (m) |
|------|--------|--------|
| 0    | −3.703 |  8.607 |
| 1    | −3.676 |  1.376 |
| 2    | −3.696 | −2.738 |
| 3    | −3.741 | −9.309 |
| 4    |  0.658 | −2.758 |
| 5    |  0.614 |  6.664 |
| 6    |  0.682 | −8.719 |

To **update the warehouse environment** (e.g. add new waypoints or change navigation paths):

1. Edit `config/warehouse_graph_fixed.geojson` — add `Point` features for new nodes and `MultiLineString` features for new edges.
2. Rebuild the package (`colcon build --packages-select titan_nav`).
3. Re-launch `route.launch.py`; the Route Server will load the updated graph automatically.

---

## Custom Nodes

### `planner_metric_node`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `publish_rate` | `1.0` | Metric logging rate (Hz) |
| `odom_topic` | `odom` | Odometry subscription |
| `plan_topic` | `plan` | Path subscription |

### `controller_dynamics_node`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `input_topic`  | `cmd_vel_raw` | Raw velocity input |
| `output_topic` | `cmd_vel`     | Filtered velocity output |
| `alpha`        | `0.5`         | Low-pass filter coefficient (0 = smooth, 1 = no filter) |
| `max_linear_x`  | `1.0` | Velocity cap (m/s) |
| `max_angular_z` | `1.5` | Angular velocity cap (rad/s) |

### `joy_node`

| Parameter | Default | Description |
|-----------|---------|-------------|
| `axis_linear`   | `1` | Joystick axis for linear velocity |
| `axis_angular`  | `3` | Joystick axis for angular velocity |
| `btn_deadman`   | `4` | Button index for deadman switch |
| `scale_linear`  | `0.5` | Linear velocity scale |
| `scale_angular` | `1.0` | Angular velocity scale |

---

## License

Apache License 2.0 — see [LICENSE](LICENSE) for details.
