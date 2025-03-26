cmake_minimum_required(VERSION 3.22)
project(mycobot_test)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

ament_package()

add_executable(move_eef_node src/move_eef.cpp)
ament_target_dependencies(move_eef_node rclcpp moveit_ros_planning_interface tf2_ros geometry_msgs)

add_executable(move_angle_node src/move_angle.cpp)
ament_target_dependencies(move_angle_node rclcpp moveit_ros_planning_interface tf2_ros geometry_msgs)

add_executable(run_node src/run.cpp)
ament_target_dependencies(run_node rclcpp moveit_core moveit_ros_planning_interface visualization_msgs geometry_msgs)
install(TARGETS run_node DESTINATION lib/${PROJECT_NAME})


install(TARGETS move_eef_node move_angle_node
  DESTINATION lib/${PROJECT_NAME}
)

