#!/bin/bash

ERRORFILE=/home/user/errorWhileAutorun
FILE=/home/user/catkin_ws/src/FloriBot4.0/floribot/launch/robot.launch

echo "Start ROS-Launch routine"
if [ -f "$ERRORFILE" ] ; then
        echo "------------ Delete old Errorfile ------------"
        rm $ERRORFILE
fi

echo "Create new Errorfile"
touch $ERRORFILE

if [ -f "$FILE" ]; then
	cd  /home/user/catkin_ws
        echo "------------ Clean up workspace ------------" >> $ERRORFILE
        #rm -r build
        #rm -r devel
        echo "------------ Build workspace ------------" >> $ERRORFILE
        export ROS_MASTER_URI="http://192.168.0.42:11311/"
        echo "ROS_MASTER_URI is set" >> $ERRORFILE
        export ROS_HOSTNAME="192.168.0.42"
        source /opt/ros/noetic/setup.bash
        catkin_make
        source /home/user/catkin_ws/devel/setup.bash
	echo "------------ Wait for network -----------" >> $ERRORFILE
        while true; do
                ifconfig | grep 192.168.0.42
                if [ $? -eq 1 ]; then
                        sleep 2 # network not up yet
                else
                        echo "------------ Network up -----------" >> $ERRORFILE
                        break # network up
                fi
        done
        echo "------------ Launch file $FILE ------------" >> $ERRORFILE
        roscore & >> $ERRORFILE
        echo "------------ roscore started ------------" >> $ERRORFILE
        roslaunch --wait floribot robot.launch >> $ERRORFILE

	#rostopic pub /engine/mode std_msgs/UInt32 "data: 1"
        exit 0
else
        echo "------------ $FILE does not exist ------------" >> $ERRORFILE
        exit 1
fi

