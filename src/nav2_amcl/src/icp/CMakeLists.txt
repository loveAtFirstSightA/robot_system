find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(csm REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

set(dependencies
  sensor_msgs
  pcl_ros
  pcl_conversions
  nav_msgs
  csm
  Eigen3
)

add_library(icp_lib plicp.cpp)
# target_include_directories(icp_lib PRIVATE include)
ament_target_dependencies(icp_lib ${dependencies})

install(TARGETS icp_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
