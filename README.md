# Terrain Mapping Drone Control

A ROS2 package for autonomous drone control with gimbal-stabilized camera and visual feature tracking capabilities using PX4 SITL simulation.

<img width="2116" height="1400" alt="image" src="https://github.com/user-attachments/assets/364db4bc-b407-41a7-8543-53ef024b3cda" />

## Overview

This package provides a complete simulation environment for testing terrain mapping and visual feature tracking algorithms. It uses a PX4-powered X500 quadcopter with a 3-axis gimbal camera in a Gazebo simulation with realistic terrain models.

### Key Features

- **Spiral Trajectory Controller**: Autonomous takeoff, descending spiral pattern (configurable diameter and descent rate), and landing
- **Gimbal Control**: Active gimbal stabilization to maintain camera pointing at ground target during flight
- **ORB Feature Tracking**: Real-time visual feature detection, tracking, and matching between consecutive frames
- **Pose Visualization**: RViz-compatible 3D visualization of drone pose with coordinate frame markers
- **Terrain Models**: Includes high-resolution planetary surface terrain meshes for realistic simulation
- **Bridge Infrastructure**: Complete ROS2-Gazebo topic bridging for camera, gimbal, and control interfaces

### Applications

- Visual odometry and SLAM algorithm testing
- Feature-based navigation research
- Gimbal control algorithm development
- Autonomous terrain mapping mission planning
- Computer vision algorithm validation
  
## Prerequisites

- ROS2 Humble
- PX4 SITL Simulator (with Gazebo)
- `px4_msgs` package
- `ros_gz_bridge` package
- OpenCV and `cv_bridge`
- Python 3.8+
- Additional Python packages: `transforms3d`, `numpy`

## Installation

1. Clone this package into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/DREAMS-lab/terrain-mapping.git terrain_mapping_drone_control
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --packages-select terrain_mapping_drone_control
```

4. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### Basic Launch

Launch the complete simulation environment (PX4 SITL, Gazebo terrain, and ROS2 bridges):
```bash
ros2 launch terrain_mapping_drone_control terrain_mapping.launch.py
```

This will:
- Start PX4 SITL with X500 gimbal-equipped drone
- Spawn the terrain model in Gazebo
- Launch ROS2-Gazebo topic bridges for camera and gimbal control

### Running Individual Nodes

**Spiral Trajectory Controller:**
```bash
ros2 run terrain_mapping_drone_control spiral_trajectory
```
The drone will:
1. Arm and switch to offboard mode
2. Take off to 20m altitude
3. Execute a descending spiral pattern (35m diameter)
4. Point gimbal at center/takeoff position throughout flight
5. Land when reaching 5m altitude

**Feature Tracker:**
```bash
ros2 run terrain_mapping_drone_control feature_tracker
```
Subscribes to `/drone_camera` and publishes annotated images with ORB features to `/feature_tracking/annotated_image`

**Pose Visualizer:**
```bash
ros2 run terrain_mapping_drone_control pose_visualizer
```
Publishes drone pose markers to `/drone/visualization_marker_array` for RViz visualization

### Visualization

**View camera feed with feature tracking:**
```bash
ros2 run rqt_image_view rqt_image_view /feature_tracking/annotated_image
```

**RViz visualization:**
```bash
rviz2 -d $(ros2 pkg prefix terrain_mapping_drone_control)/share/terrain_mapping_drone_control/config/drone_viz.rviz
```

**Monitor drone position:**
```bash
ros2 topic echo /fmu/out/vehicle_odometry
```

## Configuration

### Spiral Trajectory Parameters

Edit `spiral_trajectory.py` to modify flight behavior:

```python
self.INITIAL_HEIGHT = 20.0       # Takeoff altitude (meters)
self.SPIRAL_DIAMETER = 35.0      # Diameter of spiral pattern (meters)
self.DESCENT_RATE = 0.5          # Vertical descent speed (m/s)
self.SPIRAL_PERIOD = 10.0        # Time for one complete revolution (seconds)
self.MIN_HEIGHT = 5.0            # Minimum altitude before landing (meters)
```

### Feature Tracker Parameters

Edit `feature_tracker.py` to adjust ORB detection:

```python
nfeatures=500          # Maximum number of features to detect
scaleFactor=1.2        # Scale factor between pyramid levels
nlevels=8              # Number of pyramid levels
fastThreshold=20       # FAST detector threshold
```

### Launch Configuration

Key parameters in `terrain_mapping.launch.py`:

- `PX4_GZ_MODEL_POSE`: Initial drone spawn position (x, y, z, roll, pitch, yaw)
- Terrain model path and orientation
- ROS2-Gazebo topic bridge mappings

## Package Structure

```
terrain_mapping_drone_control/
├── config/
│   ├── drone_viz.rviz              # RViz configuration
│   └── terrain_mapping_params.yaml # Flight parameters
├── launch/
│   ├── terrain_mapping.launch.py   # Main launch file
│   └── visualization.launch.py     # RViz visualization
├── models/
│   └── terrain/                    # Terrain mesh models
│       ├── meshes/
│       │   └── artburysol175.obj   # High-res terrain mesh (67MB)
│       └── model.sdf               # Gazebo model definition
├── terrain_mapping_drone_control/
│   ├── spiral_trajectory.py        # Spiral descent controller
│   ├── feature_tracker.py          # ORB feature detection/tracking
│   └── pose_visualizer.py          # Pose visualization
├── scripts/
│   └── spiral_trajectory           # Executable script
├── package.xml
├── setup.py
└── README.md
```

## ROS2 Topics

### Published Topics

- `/fmu/in/offboard_control_mode` - Offboard control mode commands
- `/fmu/in/trajectory_setpoint` - Position/velocity setpoints
- `/fmu/in/vehicle_command` - Vehicle commands (arm, disarm, mode changes)
- `/model/x500_gimbal_0/command/gimbal_*` - Gimbal control (pitch, roll, yaw)
- `/feature_tracking/annotated_image` - Camera feed with ORB features visualized
- `/drone/visualization_marker_array` - RViz markers for drone pose
- `/drone/pose_with_covariance` - Pose with covariance for navigation stack

### Subscribed Topics

- `/fmu/out/vehicle_odometry` - Drone position and orientation
- `/fmu/out/vehicle_status` - Vehicle status information
- `/drone_camera` - RGB camera images from Gazebo
- `/drone_depth_camera` - Depth camera images from Gazebo
- `/drone_camera_info` - Camera calibration information

## License

BSD-3-Clause License

Copyright (c) 2025, Distributed Robotic Exploration and Mapping Systems Laboratory

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Citation

If you use this package in your research, please cite:

```bibtex
@software{terrain_mapping_drone_control,
  author = {DREAMS Lab},
  title = {Terrain Mapping Drone Control},
  year = {2025},
  url = {https://github.com/DREAMS-lab/terrain-mapping}
}
``` 
