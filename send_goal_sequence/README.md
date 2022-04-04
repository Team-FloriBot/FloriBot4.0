# Install
clone this repo to ros catkin workspace
```sh
cd ~/catkin_ws/src
cd ../
source opr/ros/<distro>/setup.bash
catkin_make --pkg feed_goals
```
# Run
after `move_base` is launched, use this package to send goal sequence automatically
```sh
cd ~/catkin_ws
source devel/setup.bash
roslaunch feed_goals movebase_seq.launch
``` 
# Reference
The log message catches the feedback message `GoalStatus` ([definition](http://docs.ros.org/en/api/actionlib_msgs/html/msg/GoalStatus.html)) from action server