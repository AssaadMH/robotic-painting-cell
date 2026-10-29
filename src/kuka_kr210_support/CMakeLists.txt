cmake_minimum_required(VERSION 3.8)
project(kuka_kr210_support)
find_package(ament_cmake REQUIRED)
# ROS 2 install rules only: the URDF/xacro and meshes come unchanged from ros-industrial/kuka_experimental
install(DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME})
ament_package()
