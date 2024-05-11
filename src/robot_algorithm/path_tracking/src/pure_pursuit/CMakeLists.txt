add_library(pure_pursuit_lib pure_pursuit.cpp)

ament_target_dependencies(pure_pursuit_lib
  std_msgs
  geometry_msgs
)

install(TARGETS pure_pursuit_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
