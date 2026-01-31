#!/usr/bin/env bash

echo "Killing ROS 2 and Gazebo processes..."
pkill -f slam_toolbox
pkill -f teleop_twist_keyboard
pkill -f ros2
pkill -f gz
pkill -f gzserver
pkill -f gzclient
pkill -f rviz2
pkill -f ign
pkill -f ros2_daemon
pkill -f cyclonedds
pkill -f fastdds

echo "Done."
