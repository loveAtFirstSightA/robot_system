# 设置CMake的最低版本要求
cmake_minimum_required(VERSION 3.8)

# 定义项目名称为spdlod_demo
project(spdlog_demo)

# 如果编译器为GNU C++或Clang，则添加编译选项以启用警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C标准为C99，如果未设置
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# 设置C++标准为C++17，如果未设置
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 查找并引入必要的依赖项包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

# 包含头文件目录
include_directories(
  include
)

# 设置可执行文件的名称
set(executable_name spdlog_demo)

# 设置库文件的名称
set(library_name ${executable_name}_core)

# 定义依赖项列表，供库和可执行文件使用
set(dependencies
  rclcpp
  spdlog
  fmt
)

# 添加库，并指定源文件和头文件目录
add_library(${library_name} src/spdlog_demo.cpp)
target_include_directories(${library_name} PRIVATE include)
ament_target_dependencies(${library_name} ${dependencies})

# 添加可执行文件，并指定源文件和头文件目录
add_executable(${executable_name} src/main.cpp)
target_include_directories(${executable_name} PRIVATE include)
ament_target_dependencies(${executable_name} ${dependencies})

# 将库文件链接到可执行文件中，同时显式链接fmt库以避免链接错误
target_link_libraries(${executable_name} ${library_name} fmt::fmt)

# 如果启用了测试，查找并引入自动代码检查工具
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # 以下设置用于跳过版权检查（请在添加版权和许可证后注释掉这些行）
  set(ament_cmake_copyright_FOUND TRUE)
  
  # 以下设置用于跳过cpplint检查（仅在Git仓库中有效，建议在提交代码前启用）
  set(ament_cmake_cpplint_FOUND TRUE)
  
  # 自动查找并添加测试依赖项
  ament_lint_auto_find_test_dependencies()
endif()

# 指定安装库文件的位置
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 指定安装可执行文件的位置
install(TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME}
)

# 指定安装头文件的位置
install(DIRECTORY include/
  DESTINATION include
)

# 导出库文件和依赖项，以便其他包可以使用
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)

# 生成并安装包的CMake支持文件
ament_package()
