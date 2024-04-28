# robot_system

The robot software system about slam, global path planning, local path planning.

submodule link:

    https://github.com/blackcoffeerobotics/bcr_bot.git

How to launch gazebo simulation environment?

1. ```
   sudo apt-get install ros-humble-gazebo-ros-pkgs
   ```
2. ```
   rosdep install --from-paths src --ignore-src -r -y
   ```
3. ```
   ros2 launch bcr_bot gazebo.launch.py
   ```
4. ```
   ros2 launch bcr_bot gazebo.launch.py \
   	camera_enabled:=True \
   	two_d_lidar_enabled:=True \
   	stereo_camera_enabled:=False \
   	position_x:=0.0 \
   	position_y:=0.0 \
   	orientation_yaw:=0.0 \
   	odometry_source:=world \
   	world_file:=small_warehouse.sdf \
   	robot_namespace:="bcr_bot"
   ```
