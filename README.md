# Vẽ chữ T bằng UR3e, ROS 2 Humble và MoveIt 2

Package ROS 2 điều khiển robot UR3e trong Gazebo để vẽ chữ **T** bằng chuyển động Cartesian của đầu công tác `tool0`.

## 1. Yêu cầu

- Ubuntu 22.04
- ROS 2 Humble Desktop
- Universal Robots ROS 2 Gazebo Simulation
- MoveIt 2
- Workspace `~/ros2_ws`

## 2. Cấu trúc package

```text
draw_letter_ur/
├── include/
│   └── draw_letter_ur/
├── launch/
│   └── draw_letter.launch.py
├── src/
│   ├── draw_letter.cpp
│   ├── moveit_probe.cpp
│   └── safe_pose.cpp
├── CMakeLists.txt
├── package.xml
└── README.md