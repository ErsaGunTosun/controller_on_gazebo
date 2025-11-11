# Controller on Gazebo

A ROS2 package for controlling a differential drive robot in Gazebo using a game controller (Xbox/PlayStation controller). This package enables real-time teleoperation of a TurtleBot3 Waffle Pi inspired robot in simulation.

## Overview

This package provides a complete integration between ROS2, Gazebo, and game controllers, allowing you to:
- Spawn and simulate a differential drive robot in Gazebo
- Control the robot using an Xbox controller or similar joystick
- Bridge ROS2 topics with Gazebo messages
- Publish robot state and odometry information

## Features

- **Robot Model**: TurtleBot3 Waffle Pi inspired differential drive robot with:
  - Two driven wheels
  - Caster wheels for stability
  - LiDAR sensor mounting point
  - Camera mounting point
  - IMU sensor

- **Controller Support**: Real-time control via game controllers with configurable scaling parameters

- **Gazebo Integration**: Full Gazebo simulation with differential drive plugin

- **ROS2 Bridge**: Seamless communication between ROS2 and Gazebo for:
  - Command velocity (`/cmd_vel`)
  - Odometry data (`/odom`)
  - Clock synchronization

## Prerequisites

- **ROS2** (tested on Humble/Iron)
- **Gazebo** (Gazebo Fortress or later)
- **ros_gz_sim** and **ros_gz_bridge** packages
- **joy** package for joystick support
- A compatible game controller (Xbox, PlayStation, or generic)

### Install Dependencies

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-gazebo-ros-pkgs
sudo apt install ros-${ROS_DISTRO}-ros-gz
sudo apt install ros-${ROS_DISTRO}-joy
```

## Installation

1. Create a workspace (if you don't have one):

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone this repository:

```bash
git clone <repository-url> controller_on_gazebo
```

3. Build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select controller_on_gazebo
source install/setup.bash
```

## Usage

### Launch Everything at Once

To launch Gazebo with the robot and controller support:

```bash
ros2 launch controller_on_gazebo controller_up.launch.py
```

This will start:
- Gazebo server and client
- Robot state publisher
- Robot spawner
- Joy node for controller input

**Important**: After launching, you need to start the controller node in a separate terminal:

```bash
ros2 run controller_on_gazebo xbox_controller
```

### Launch Without Controller

To launch just the simulation without controller support:

```bash
ros2 launch controller_on_gazebo bring_up.launch.py
```

### Manual Launch Steps

You can also launch components individually:

1. **Robot State Publisher**:
```bash
ros2 launch controller_on_gazebo robot_state_publisher.launch.py
```

2. **Spawn Entity in Gazebo**:
```bash
ros2 launch controller_on_gazebo spawn_entity.launch.py
```

3. **Controller Node** (if launching manually):
```bash
ros2 run controller_on_gazebo xbox_controller
```

4. **Joy Node** (for controller input):
```bash
ros2 run joy joy_node
```

## Controller Mapping

The default controller mapping uses:
- **Left Stick (Y-axis)**: Linear velocity (forward/backward)
- **Left Stick (X-axis)**: Angular velocity (rotation)

### Controller Configuration

The controller node accepts the following parameters:

- `scale_linear` (default: 0.5): Scaling factor for linear velocity
- `scale_angular` (default: 1.0): Scaling factor for angular velocity

You can modify these parameters by editing the launch file or passing them at runtime:

```bash
ros2 run controller_on_gazebo xbox_controller --ros-args -p scale_linear:=0.7 -p scale_angular:=1.5
```

### Joy Node Configuration

The joy node is configured with:
- Device: `/dev/input/js0`
- Deadzone: 0.05
- Autorepeat rate: 20.0 Hz

Modify these in `controller_up.launch.py` if needed.

## Package Structure

```
controller_on_gazebo/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package metadata and dependencies
├── src/
│   └── xbox_controller.cpp     # Controller node source code
├── launch/
│   ├── bring_up.launch.py      # Launch simulation without controller
│   ├── controller_up.launch.py # Launch simulation with controller
│   ├── robot_state_publisher.launch.py
│   └── spawn_entity.launch.py
├── urdf/
│   ├── robot.urdf.xacro        # Robot description
│   └── robot.gazebo.xacro      # Gazebo-specific configurations
├── models/
│   └── robot/
│       ├── meshes/             # 3D mesh files for visualization
│       └── model.config
└── worlds/
    └── empty_world.world       # Gazebo world file
```

## Topics

### Published Topics

- `/cmd_vel` (geometry_msgs/Twist): Velocity commands to the robot
- `/odom` (nav_msgs/Odometry): Robot odometry from Gazebo
- `/clock` (rosgraph_msgs/Clock): Simulation time

### Subscribed Topics

- `/joy` (sensor_msgs/Joy): Joystick input from the controller

## Nodes

### xbox_controller

A ROS2 node that converts joystick input to velocity commands.

**Parameters:**
- `scale_linear` (double): Linear velocity scaling factor
- `scale_angular` (double): Angular velocity scaling factor

**Subscribed Topics:**
- `/joy` (sensor_msgs/Joy)

**Published Topics:**
- `/cmd_vel` (geometry_msgs/Twist)

## Troubleshooting

### Controller Not Detected

If your controller is not detected:

1. Check if the device exists:
```bash
ls -l /dev/input/js*
```

2. Test the controller:
```bash
ros2 run joy joy_node
ros2 topic echo /joy
```

3. If the device is at a different path, update the `dev` parameter in `controller_up.launch.py`

### Robot Not Moving

1. Check if the joy messages are being published:
```bash
ros2 topic echo /joy
```

2. Check if velocity commands are being sent:
```bash
ros2 topic echo /cmd_vel
```

3. Verify the bridge is running:
```bash
ros2 topic list | grep cmd_vel
```

### Gazebo Not Starting

Make sure you have the correct Gazebo version installed and ros_gz packages:

```bash
ign gazebo --version
ros2 pkg list | grep ros_gz
```

**Note**: This package uses Ignition Gazebo (accessed via `ign` command), not the newer `gz` command or Classic Gazebo.

## Customization

### Changing Robot Parameters

Edit `urdf/robot.gazebo.xacro` to modify:
- Wheel separation and radius
- Maximum accelerations
- Torque limits
- Odometry publish frequency

### Changing Controller Sensitivity

Modify the parameters in the controller node launch configuration or the source code in `src/xbox_controller.cpp`.

### Using Different World Files

Update the `world_path` variable in the launch files to point to your custom world file.
