#!/bin/bash
# installs Gazebo Harmonic + ROS 2 bridge + ros2_control plugin (Jazzy)
set -e
export DEBIAN_FRONTEND=noninteractive
sudo apt-get update -q
sudo apt-get install -y -q ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-controllers ros-jazzy-joint-trajectory-controller mesa-utils
echo INSTALL_DONE
