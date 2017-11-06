# Netbook Setup Script
# Use this script on the netbook to setup the environment and
# necessary scripts to run the robot

# Setup ROS environment
cd ~/catkin_ws
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT_3D_SENSOR=kinect
export ROS_MASTER_URI="http://$(hostname -I):11311"

# Don't lock when lid closes
LID_SWITCH="$(cat /etc/systemd/logind.conf | grep 'HandleLidSwitch=ignore')"
if [ -z "$LID_SWITCH" ]; then
    sudo echo "HandleLidSwitch=ignore" > /etc/systemd/logind.conf
fi

# Don't sleep
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

# Spawn two terminals each running minimal build and kinect sensor code on netbook
xterm -hold -e "cd ~/catkin_ws; source devel/setup.bash; roslaunch turtlebot_bringup minimal.launch" &
sleep 15s
xterm -hold -e "cd ~/catkin_ws; source devel/setup.bash; roslaunch turtlebot_bringup 3dsensor.launch" &

#Echo static IP for connection of other laptops
echo "Static IP to connect to: $(hostname -I)"
