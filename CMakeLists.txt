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
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Strike.action"
  "action/Recover.action"
  DEPENDENCIES action_msgs
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(striker_ci src/striker_ci.cpp)
ament_target_dependencies(striker_ci
  rclcpp
  rclcpp_action
  px4_msgs
  sensor_msgs
  geometry_msgs
  Eigen3
  px4_ros2_cpp
)
rosidl_target_interfaces(striker_ci ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  striker_ci
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/send_actions.py
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
