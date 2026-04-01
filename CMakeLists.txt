cmake_minimum_required(VERSION 3.8)
project(px4_ros2_striker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(striker_node src/striker_node.cpp)
ament_target_dependencies(striker_node
  rclcpp
  px4_msgs
  sensor_msgs
  geometry_msgs
  Eigen3
)

add_executable(rviz_path_publisher src/rviz_path_publisher.cpp)
ament_target_dependencies(rviz_path_publisher
  rclcpp
  px4_msgs
  geometry_msgs
  nav_msgs
)

install(TARGETS
  striker_node
  rviz_path_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
