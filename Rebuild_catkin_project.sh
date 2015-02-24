#!/bin/bash
echo "Rebuilding ROS catkin project"
echo ""
echo "Contents of ~/ : "
ls ~/
echo ""
echo "Input the directory of your catkin workspace relative to ~/ (usually catkin_ws):"
read DIRECTORY
cd ~/$DIRECTORY
source ./devel/setup.bash		#Source the catkin workspace

echo "Creating log directory, in case it does not exist"
mkdir ~/OttershawLogs

echo "Cleaning catkin build...."
timestamp=$( date +"%m-%d-%Y+%T" )
logname="CatkinClean$timestamp"
touch ~/OttershawLogs/$logname.log
catkin_make clean > ~/OttershawLogs/$logname.log
echo "END OF LOG" >> ~/OttershawLogs/$logname.log
echo "Done"
echo

echo "Building catking workspace (this may take some time)...."
timestamp=$( date +"%m-%d-%Y+%T")
logname="CatkinBuild$timestamp"
touch ~/OttershawLogs/$logname.log
catkin_make --pkg ottershaw_masta > ~/OttershawLogs/$logname.log
echo "END OF LOG" >> ~/OttershawLogs/$logname.log
echo "Done"
echo
