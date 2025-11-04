#!/bin/bash
source /opt/ros/jazzy/setup.bash
cp -r ./AutSys_Labs_ROS2_Testframework/Assignment_2/testframework ros2_ws/src
cd ros2_ws
colcon build --cmake-args -DTEST_FLAG=OFF
