#!/usr/bin/bash

# ros2 run bcr_teleop bcr_teleop_node.py

source /home/lio/project_repository/robot_system/install/local_setup.bash

ros2 run bcr_teleop bcr_teleop_node.py --ros-args --remap cmd_vel:=cmd_vel
