#!/bin/bash
echo "Updating Arduino ROS libraries to include new message types"
echo ""
echo "Contents of ~/ : "
ls ~/
echo ""
echo "Input the directory of your catkin workspace relative to ~/ (usually catkin_ws):"
read DIRECTORY
cd ~/$DIRECTORY

echo "Creating log directory, in case it does not exist"
mkdir ~/OttershawLogs

echo "Removing previous Arduino ROS libraries prior to rebuild"
timestamp=$( date +"%m-%d-%Y+%T" )
logname="ArduinoMsgBuild$timestamp"
rm -r ~/ProjectOttershaw/Arduino/libraries/ros_lib/
rosrun rosserial_arduino make_libraries.py /home/eljefe/ProjectOttershaw/Arduino/libraries/ src/ottershaw_masta/ > ~/OttershawLogs/$logname.log
echo "Note: it is normal for geometry_msgs, sensor_msgs, shape_msgs to fail to generate libraries"
echo "   You may need to close/open your Arduino IDE for the libraries to be updated"
echo ""

