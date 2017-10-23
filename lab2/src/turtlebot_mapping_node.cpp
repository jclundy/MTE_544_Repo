//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 1
// It outlines the basic setup of a ros node and the various 
// inputs and outputs.
// 
// Author: James Servos 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

void scan_callback(const sensor_msgs::LaserScan& scan)
{
	//This function is called when a new LaserScan is received

}


int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"mapping_node");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/scan", 1, scan_callback);

    //Setup topics to Publish from this node
	ros::Publisher map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);
	
	//Occupancy Grid Variable
	nav_msgs::OccupancyGrid occupancyGrid;	
	    
	//Set the loop rate
    ros::Rate loop_rate(30);    //30Hz update rate

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages
    
    	//Main loop code goes here:

    	map_publisher.publish(occupancyGrid); // Publish the command velocity
    }

    return 0;
}
