# MTE_544_Repo
## How to setup
1. cd ~/catkin_ws/src
2. git clone https://github.com/jclundy/MTE_544_Repo.git
3. cd ../ 
4. catkin_make

## Running Turtlebot and Mapping in Simulation
- rosrun turtlebot_pos_client 
- turtlebot_pos_client_node

to successfully run sim, follow these steps:
1) launch gazebo     	roslaunch turtlebot_example turtlebot_gazebo.launch
2) launch rviz       	roslaunch turtlebot_rviz_launchers view_navigation.launch
3) launch amcl		roslaunch turtlebot_example amcl_demo.launch map_file:=/home/jacob/TARosMap.yaml
4) launch teleop	roslaunch turtlebot_teleop keyboard_teleop.launch

## How to run Lab 1 simulation
1. Run simulator
roslaunch lab1 turtlebot_gazebo.launch
2. Run AMCL demo node, with saved map
roslaunch lab1 amcl_demo.launch map_file:=/home/YOUR_NAME/catkin_ws/src/MTE_544_Repo/prebuilt_maps/new_map.yaml
3. Run square driving code square_node
rosrun lab1 square_node

## How to run Lab 2 mapping node
1. Run simulator:
roslaunch lab1 turtlebot_gazebo.launch
2. Run mapping node:
rosrun lab2 turtlebot_mapping_node
3. Run rviz:
rosrun rviz rviz
4. In rviz window, add a map; topic for map is '/map'
