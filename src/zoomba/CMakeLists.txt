cmake_minimum_required(VERSION 3.8)
project(zoomba)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Diff drive controller
add_executable(diff_drive_controller src/diff_drive_controller.cpp)
target_include_directories(diff_drive_controller PUBLIC include)
ament_target_dependencies(diff_drive_controller
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

install(TARGETS
  diff_drive_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/publisher.py
  src/subscriber.py
  src/lidar.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  urdf
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()