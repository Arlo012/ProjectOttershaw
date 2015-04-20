#!/bin/bash
echo "This file should be run if there are dependency erros after rebuilding a catkin project."
echo "Proceed? (y/n)"
read response

if [ "$response" == "y" ]; then
	echo "Resolving indirect dependencies (see http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage for details)"
	rospack depends1 rospy
else
	echo "Operation aborted"
fi
