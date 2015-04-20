#!/bin/bash
echo "This script will source your machine's catkin workspace to run custom ottershaw_masta code (e.g. custom message types)."
echo "Proceed? (y/n)"
read response

if [ "$response" == "y" ]; then
	echo "Contents of ~/ : "
	ls ~/
	echo ""
	echo "Input the directory of your catkin workspace relative to ~/ (usually catkin_ws):"
	read DIRECTORY
	cd ~/$DIRECTORY
	source ./devel/setup.bash		#Source the catkin workspace
	echo "Sourced your catkin workspace at $DIRECTORY"
else
	echo "Operation aborted"
fi
