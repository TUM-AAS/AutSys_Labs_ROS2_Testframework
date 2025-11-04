#!/bin/bash
if [ $1 -eq 1 ]
then
	source /opt/ros/jazzy/setup.bash
	cd ros2_ws
	source install/setup.bash
	ros2 launch testframework test.launch.py
	cd ../AutSys_Labs_ROS2_Testframework/Assignment_2/testing
	mkdir build
	cd build
	cmake ..
	make
	cd ../../../..
fi
file="results.txt"
echo ""
echo ""
if [ -f "$file" ]
then
	cat $file
else
	echo "No results available."
fi
./AutSys_Labs_ROS2_Testframework/Assignment_2/testing/build/runTest --gtest_filter=$2