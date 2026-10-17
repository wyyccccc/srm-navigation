# SRM Navigation

ROS 2 navigation project for SRM / RoboMaster robot.

This repository integrates LiDAR localization, Nav2 navigation,
real-robot TF handling, and upper-to-lower controller communication.

## Environment

- Ubuntu 22.04
- ROS 2 Humble
- Livox Mid-360
- Nav2
- Point-LIO
- Intel NUC

## Main Features

- Livox Mid-360 driver integration
- Point-LIO localization
- Nav2 autonomous navigation
- Real-robot TF integration
- Obstacle avoidance
- Waypoint navigation
- Serial communication with lower controller
- Velocity command watchdog and safety stop

## My Modifications

This project includes the following integration and modifications:

- Real Mid-360 configuration
- Mid-360 IP configuration for `192.168.1.184`
- Point-LIO gravity correction
- Real robot TF debugging and correction
- `sensor_scan_generation` TF lookup changed to use latest available transform
- Added `srm_nav_task`
- Added waypoint navigation script
- Added Nav2 goal sending script
- Added serial bridge for chassis control
- Implemented CRC8 / CRC16 serial protocol
- Added command watchdog and zero-velocity safety handling

## Build

```bash
cd ~/srm-navigation
colcon build
source install/setup.bash
```

## ROS Domain

For the current robot setup:
```bash
export ROS_DOMAIN_ID=42
```

## Simulation

```bash
source ~/srm-navigation/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch pb2025_nav_bringup \
rm_navigation_simulation_launch.py \
world:=rmuc_2025 \
slam:=True
```

## Real Robot

```bash
source ~/srm-navigation/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch pb2025_nav_bringup \
rm_navigation_reality_launch.py \
slam:=True \
use_robot_state_pub:=True
```

## Serial Bridge

```bash
ros2 run srm_nav_task serial_bridge
```

Current serial configuration:

- Device: /dev/ttyACM0
- Baud rate: 115200
- Update rate: 100 Hz
- ROS topic: /red_standard_robot1/cmd_vel

## Waypoint Navigation

```bash
ros2 run srm_nav_task follow_waypoints
```

## Project Structure

```text
srm-navigation/
├── pb2025_nav_bringup/
├── point_lio/
├── loam_interface/
├── sensor_scan_generation/
├── livox_ros_driver2/
├── pb_nav2_plugins/
├── pb_omni_pid_pursuit_controller/
├── pointcloud_to_laserscan/
├── small_gicp_relocalization/
├── srm_nav_task/
└── README.md
```

## Credits

This project integrates and modifies selected components from
PB2025 Sentry Navigation and other open-source ROS 2 projects.
Original licenses, copyright notices, and attribution are retained
in the corresponding source directories and files.
PB2025 Sentry Navigation:
https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav
License
Please refer to the license files included in each third-party component.
