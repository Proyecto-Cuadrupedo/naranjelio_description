# naranjelio_description

This repository contains the robot description package for Naranjelio, adapted for ROS 2 Jazzy with RViz visualization and Gazebo (ros_gz_sim) simulation.

## Dependencies

To run this package, install ROS 2 Jazzy and these packages:

- `xacro`
- `robot_state_publisher`
- `joint_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`
- `ros_gz_sim`
- `ros_gz_bridge`
- `ros2_control`
- `ros2_controllers`
- `controller_manager`

System packages (Ubuntu 24.04 + ROS 2 Jazzy):

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-rviz2 \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager
```

## Installation

This should get your workspace ready:

```bash
mkdir -p naranjelio_ws/src
cd naranjelio_ws/src

# Clone your package
git clone <your-repo-url>/naranjelio_description.git

cd ..
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Usage

### 1) Visualize in RViz

```bash
source /opt/ros/jazzy/setup.bash
source naranjelio_ws/install/setup.bash
ros2 launch naranjelio_description display.launch.py
```

Use `gui:=False` if you prefer non-GUI joint state publisher:

```bash
ros2 launch naranjelio_description display.launch.py gui:=False
```

### 2) Launch Gazebo simulation

```bash
source /opt/ros/jazzy/setup.bash
source naranjelio_ws/install/setup.bash
ros2 launch naranjelio_description gazebo.launch.py
```

This launch file:

- Starts Gazebo through `ros_gz_sim`
- Publishes robot description from `naranjelio.xacro`
- Bridges `/clock` from Gazebo to ROS 2
- Spawns the robot in simulation
- Loads `joint_state_broadcaster`
- Loads `joint_trajectory_controller`

## Notes

- The main robot description is in `urdf/naranjelio.xacro`.
- If you edit Xacro/URDF, rebuild with `colcon build --symlink-install`.
- Meshes are loaded from the `meshes/` folder.
- Controller parameters are in `config/ros2_controllers.yaml`.

