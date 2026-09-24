cmake_minimum_required(VERSION 3.8)
project(multi_modal_robot_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

install(
  DIRECTORY urdf meshes config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()