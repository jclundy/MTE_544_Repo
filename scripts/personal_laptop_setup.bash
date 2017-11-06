# Arguments:
# 1st argument - SIM or ROB
# 2nd argument - IP address
# example: `bash lab2.sh ROB 192.168.1.1

# NOTE: Requires edits for more robustness, run to local environment call required

cd ~/catkin_ws
source /opt/ros/kinect/setup.bash
source ~/catkin_ws/devel/setup.bash
 

RUN_OPT="$1"
echo $RUN_OPT
if [ -z "$RUN_OPT" ]; then
	RUN_OPT="SIM"
fi

if [ "$RUN_OPT" == "SIM" ]; then
	echo "Running simulation" &
	xterm -hold -e "cd ~/catkin_ws; source devel/setup.bash; roslaunch turtlebot_example turtlebot_gazebo.launch" &
	sleep 15s
elif [ "$RUN_OPT" == "ROB" ]; then
    if [ -n "$2" ]; then
        echo "Requires IP of Netbook"
    else
	    echo "Running for robot"
	    export TURTLEBOT_3D_SENSOR=kinect
	    export ROS_MASTER_URI="http://$2:11311"
	    export ROS_IP="$2"
    fi
else
	echo "Option undefined"
fi
xterm -hold -e "cd ~/catkin_ws; source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
