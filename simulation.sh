#!/usr/bin/bash

ros2 launch bcr_bot gazebo.launch.py

# ros2 launch bcr_bot gazebo.launch.py \
# 	camera_enabled:=True \
# 	two_d_lidar_enabled:=True \
# 	stereo_camera_enabled:=False \
# 	position_x:=1.760 \
# 	position_y:=0.0 \
# 	orientation_yaw:=-1.5707 \
# 	odometry_source:=world \
# 	world_file:=small_warehouse.sdf \
# 	robot_namespace:="bcr_bot"
