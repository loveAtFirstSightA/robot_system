#!/usr/bin/bash

# ros2 launch nav2_amcl amcl.launch.py

ros2 launch nav2_amcl amcl.launch.py use_sim_time:=true map_yaml_path:=/home/lio/project_repository/robot_system/maps/factory.yaml
