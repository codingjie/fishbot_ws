cmake_minimum_required(VERSION 3.8)
project(fishbot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 安装 launch 和 urdf 目录
install(
  DIRECTORY launch urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

add_executable(rotate_wheel src/rotate_wheel.cpp)
ament_target_dependencies(rotate_wheel rclcpp sensor_msgs)
install(TARGETS
  rotate_wheel
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(wall_follower src/wall_follower.cpp)
ament_target_dependencies(wall_follower rclcpp sensor_msgs geometry_msgs)
install(TARGETS 
  wall_follower
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
