#!/usr/bin/bash

# ros2 run bcr_teleop bcr_teleop_node.py

ros2 run bcr_teleop bcr_teleop_node.py --ros-args --remap cmd_vel:=cmd_vel
