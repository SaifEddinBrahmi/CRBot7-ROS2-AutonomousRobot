
# CRBot7 - Autonomous Navigation Robot 🇵🇹

**CR7-inspired autonomous mobile robot with Portugal colors - Because robots can have legendary moves too!** ⚽✨

*"SIUUUU!"* - Just like Cristiano Ronaldo dominates the field, CRBot7 dominates autonomous navigation! 🚀

[![GitHub](https://img.shields.io/badge/GitHub-CRBot7--ROS2--AutonomousRobot-blue?logo=github)](https://github.com/SaifEddinBrahmi/CRBot7-ROS2-AutonomousRobot)

Developed by: **Saifeddin Brahmi** (saifeddin.brahmi@ensi-uma.tn)

🔗 **Repository:** [https://github.com/SaifEddinBrahmi/CRBot7-ROS2-AutonomousRobot](https://github.com/SaifEddinBrahmi/CRBot7-ROS2-AutonomousRobot)

---

## 🤖 About CRBot7

**CRBot7** is a high-performance differential drive mobile robot inspired by Portugal's greatest legend, Cristiano Ronaldo (CR7). Just like CR7's precision on the field, CRBot7 delivers:

- 🔴 **Portugal Flag Colors**: Red chassis and green wheels representing Portuguese pride
- 🎯 **Precision Navigation**: SLAM mapping and Nav2 path planning for warehouse automation
- 📹 **Vision System**: 640x480 camera at 20Hz for real-time environment monitoring
- 🌀 **360° Awareness**: Lidar sensor with 3.5m range for obstacle detection
- ⚡ **Legendary Performance**: Optimized controllers with TimerAction delays for reliable operation

**SIUUUU!** 🎉 - This robot scores goals in autonomous navigation! ⚽🥅

---

## ✨ Key Features

- 🏆 **CL Performance**: Built with ROS2 Humble and Ignition Gazebo Fortress
- 🇵🇹 **Portugal Theme**: Red & Green color scheme honoring CR7's homeland
- 🗺️ **Real-time SLAM**: Asynchronous mapping with slam_toolbox
- 🧭 **Full Nav2 Stack**: DWB local planner with intelligent path planning
- 🎮 **Keyboard Control**: Drive it like CR7 controls a football
- 📷 **Live Camera Feed**: See what the robot sees in real-time
- 🔧 **Modular Design**: Clean xacro-based URDF for easy customization

## 📋 Requirements
- **ROS 2 Humble** (Ubuntu 22.04)
- **Ignition Gazebo Fortress**
- **Navigation2** stack
- **slam_toolbox** for SLAM mapping
- **ros2_control** with ign_ros2_control plugin

## 🚀 Quick Start

### 1. Install Dependencies
```bash
# Install Navigation2 and SLAM Toolbox
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox

# Install Ignition Gazebo and ros2_control packages
sudo apt install ros-humble-ros-ign ros-humble-ign-ros2-control

# Install dependencies from workspace
cd ~/ros2_ws
rosdep install -y -r -i --from-paths src --ignore-src
```

### 2. Build the Project
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Build CRBot7 package
cd ~/ros2_ws
colcon build --packages-select crbot7 --symlink-install

# Source the workspace
source install/setup.bash
```

### 3. Launch the Complete System
```bash
# Launch Gazebo simulation + Controllers + SLAM + Nav2 + RViz
ros2 launch crbot7 complete_system.launch.py world_file:=empty.sdf

# Alternative: Use bookstore world (requires AWS models)
ros2 launch crbot7 complete_system.launch.py world_file:=bookstore.sdf
```

### 4. Control the Robot

**Keyboard Teleoperation:**
```bash
# In a new terminal, launch keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Autonomous Navigation:**
1. Drive the robot around with keyboard to build a map
2. In RViz, click the **"2D Goal Pose"** button in the toolbar
3. Click on the map to set the goal location
4. Drag to set the goal orientation
5. Watch CRBot7 navigate autonomously!

## 🎯 Features

### Modular Robot Description
The robot is split into clean, maintainable xacro files:
- `robot_core.xacro` - Chassis, wheels, caster, and IMU
- `lidar.xacro` - 360° GPU Lidar sensor
- `camera.xacro` - RGB camera sensor
- `ros2_control.xacro` - Controller interfaces
- `inertial_macros.xacro` - Inertial calculation utilities
- `crbot7.urdf.xacro` - Main robot assembly

### Navigation Stack
- **SLAM**: slam_toolbox with online async mapping
- **Localization**: AMCL for pose estimation
- **Path Planning**: Nav2 with DWB local planner
- **Costmaps**: Local and global costmaps with inflation layers
- **Recovery Behaviors**: Backup, spin, wait behaviors

### Controller Configuration
- **Joint State Broadcaster**: Publishes joint states for RViz
- **Differential Drive Controller**: Velocity control with odometry
- **Optimized Timing**: TimerAction delays for reliable controller loading

## 📁 Project Structure

```
CRBot7-ROS2-AutonomousRobot/
├── config/                          # Configuration files
│   ├── controllers.yaml             # ros2_control controller config
│   ├── ekf.yaml                     # Extended Kalman Filter
│   └── nav2_params.yaml             # Navigation2 parameters
├── description/                     # Modular robot URDF
│   ├── crbot7.urdf.xacro            # Main robot description
│   ├── robot_core.xacro             # Base, wheels, caster, IMU
│   ├── lidar.xacro                  # Lidar sensor (360°, 3.5m range)
│   ├── camera.xacro                 # Camera sensor (640x480 @ 20Hz)
│   ├── ros2_control.xacro           # Control interfaces
│   └── inertial_macros.xacro        # Inertial calculation utilities
├── launch/                          # Launch files
│   ├── complete_system.launch.py   # Full system launcher
│   └── display.launch.py            # Gazebo + Controllers + RViz
├── models/                          # Gazebo models
│   └── TurtlebotArena/              # Example arena model
├── rviz/                            # RViz configurations
│   ├── navigation_config.rviz       # Navigation visualization
│   └── urdf_config.rviz             # Robot model visualization
├── scripts/                         # Python navigation scripts
│   ├── follow_waypoints.py          # Multi-waypoint navigation
│   └── reach_goal.py                # Single goal navigation
├── worlds/                          # Gazebo world files
│   ├── empty.sdf                    # Empty world
│   └── bookstore.sdf                # Bookstore environment
├── .gitignore                       # Git ignore rules
├── CMakeLists.txt                   # Build configuration
├── package.xml                      # ROS2 package metadata
└── README.md                        # This file
```

## 🎮 Available Launch Files

### Complete System (Recommended)
```bash
ros2 launch crbot7 complete_system.launch.py world_file:=empty.sdf
```
Launches: Gazebo + Controllers + SLAM Toolbox + Nav2 + RViz

### Basic Display Only
```bash
ros2 launch crbot7 display.launch.py world_file:=empty.sdf
```
Launches: Gazebo + Controllers + RViz (no navigation)

## 🛠️ Advanced Usage

### Programmatic Navigation
```bash
# Navigate to a single goal pose
ros2 run crbot7 reach_goal.py

# Follow multiple waypoints
ros2 run crbot7 follow_waypoints.py
```

### Save Your Map
```bash
# After building a good map, save it
ros2 run nav2_map_server map_saver_cli -f ~/my_warehouse_map
```

### Load a Saved Map
Modify `nav2_params.yaml` to use your saved map instead of SLAM.

## 🎨 Portugal Theme Colors

The robot features Portugal flag colors:
- **Chassis**: Red (`rgba="0.8 0.0 0.0 1.0"`)
- **Wheels**: Green (`rgba="0.0 0.6 0.0 1.0"`)
- **Caster**: Gray (`rgba="0.5 0.5 0.5 1.0"`)

Colors are applied in both visual materials and Gazebo materials for consistency.

## 📝 Configuration Notes

### Controller Timing
TimerAction delays are crucial for reliable controller loading:
- 3s delay before loading `joint_state_broadcaster`
- 2s delay before loading `diff_drive_base_controller`

### RViz Optimization
All navigation topics use **Filter size: 10** for smooth visualization:
- LaserScan, Map, Path, Local/Global Costmaps, Goal poses

### Topic Relays
Automatic topic relays connect teleop to controller:
- `/cmd_vel` → `/diff_drive_base_controller/cmd_vel_unstamped`
- `/diff_drive_base_controller/odom` → `/odom`

## 🐛 Troubleshooting

**Controllers timeout error?**
- Ensure TimerAction delays are present in `display.launch.py`
- Check that Gazebo is fully loaded before controller spawn

**RViz not launching?**
- Verify RViz config exists in `rviz/` directory
- Check that SLAM Toolbox launches before RViz

**Robot not moving with keyboard?**
- Verify teleop is publishing to `/cmd_vel`
- Check topic relay nodes are running
- Ensure controllers are loaded: `ros2 control list_controllers`

**No map building?**
- Check lidar data: `ros2 topic echo /scan`
- Verify slam_toolbox is running: `ros2 node list`
- Drive robot around to gather lidar data

## 📚 Resources

- [Navigation2 Documentation](https://navigation.ros.org/)
- [slam_toolbox GitHub](https://github.com/SteveMacenski/slam_toolbox)
- [Ignition Gazebo Documentation](https://gazebosim.org/docs)
- [ros2_control Documentation](https://control.ros.org/)

## 📄 License

Apache License 2.0

## 👤 Author

**Saifeddin Brahmi**  
Email: saifeddin.brahmi@ensi-uma.tn  
GitHub: [@SaifEddinBrahmi](https://github.com/SaifEddinBrahmi)

---

## ⭐ Show Your Support

If you found this project useful or if you're a **CR7 fan** ⚽🇵🇹, please consider giving it a star on GitHub!

[![Star on GitHub](https://img.shields.io/github/stars/SaifEddinBrahmi/CRBot7-ROS2-AutonomousRobot?style=social)](https://github.com/SaifEddinBrahmi/CRBot7-ROS2-AutonomousRobot)

**⭐ Star this repo if:**
- ✅ You found it useful for learning ROS2 navigation
- ✅ You appreciate the Portugal theme 🇵🇹
- ✅ You're a CR7 fan! ⚽ *SIUUUU!*
- ✅ You want to support open-source robotics

---

*Built with ROS 2 Humble, Ignition Gazebo Fortress, and Navigation2 🚀*

