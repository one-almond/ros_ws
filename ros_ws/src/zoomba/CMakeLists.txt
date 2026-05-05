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



install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/publisher.py
  src/subscriber.py
  src/diff_drive.py
  src/odom_pub.py
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