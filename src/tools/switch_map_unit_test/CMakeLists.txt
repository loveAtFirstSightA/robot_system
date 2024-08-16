cmake_minimum_required(VERSION 3.8)
project(switch_map_unit_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fcbox_msgs REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

include_directories(
  include
)

set(executable_name switch_map_unit_test)
set(executable_client change_state)

set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  fcbox_msgs
  spdlog
  fmt
)

add_library(${library_name} src/switch_map_unit_test.cpp)
target_include_directories(${library_name} PRIVATE include)
ament_target_dependencies(${library_name} ${dependencies})

add_executable(${executable_name} src/main.cpp)
target_include_directories(${executable_name} PRIVATE include)
ament_target_dependencies(${executable_name} ${dependencies})

target_link_libraries(${executable_name} ${library_name} fmt::fmt)

add_executable(${executable_client} src/change_state.cpp)
target_include_directories(${executable_client} PRIVATE include)
ament_target_dependencies(${executable_client} ${dependencies})

target_link_libraries(${executable_client} ${library_name} fmt::fmt)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${executable_client}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_package()
