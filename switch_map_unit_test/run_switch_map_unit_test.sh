#!/bin/bash

source /opt/ros/humble/setup.bash

source /home/fcbox/catkin_ws/install/local_setup.bash

# 获取当前系统日期和时间
current_date_time=$(date +"%Y-%m-%d_%H-%M-%S")
log_file="switch_map_unit_test_log_$current_date_time.log"

# 后台运行并将日志重定向
nohup ./switch_map_unit_test > "$log_file" 2>&1 &

echo "Script is running in the background. Log is saved in $log_file"

