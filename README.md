# MTE_544_Repo
## How to setup
1. cd ~/catkin_ws/src
2. git clone https://github.com/jclundy/MTE_544_Repo.git
3. catkin_make
- note that you may need to remove the src/example_node folder 
## How to run Lab 1 simulation
1. Run simulator
roslaunch turtlebot_example turtlebot_gazebo.launch
2. Run AMCL demo node, with saved map
roslaunch turtlebot_example amcl_demo.launch map_file:='ABSOLUTE_PATH_TO_map.yaml
3. Run turtlebot_example_node
rosrun turtlebot_example turtlebot_example_node
