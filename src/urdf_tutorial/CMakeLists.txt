cmake_minimum_required(VERSION 3.8)
project(urdf_tutorial)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(ros2talker tutorial/ros2talker.cpp)
ament_target_dependencies(ros2talker rclcpp geometry_msgs std_msgs trajectory_msgs sensor_msgs tf2 tf2_ros)

install(TARGETS
  ros2talker
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch rviz2 xacro meshes urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
