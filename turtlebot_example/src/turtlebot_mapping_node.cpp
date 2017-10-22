//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various 
// inputs and outputs needed for this lab
// 
// Author: James Servos 
// Edited: Nima Mohajerin
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

const double MAP_X = 5; //meters
const double MAP_Y = 5; //meters
const double MAP_SIZE = 5;
const double GRID_RES = 0.1; //meters 
const double SCALE = MAP_SIZE/GRID_RES;
const int scanSize = 134;

ros::Publisher pose_publisher;
ros::Publisher marker_pub;
double currentScanVector[134];
sensor_msgs::LaserScan currentScan; 

double ips_x;
double ips_y;
double ips_yaw;

short sgn(int x) { return x >= 0 ? 1 : -1; }

int convertDistanceToGridCoordinates(double reading)
{
	return int(reading * SCALE);
}

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg) 
{

    int i;
    for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

    ips_x = msg.pose[i].position.x ;
    ips_y = msg.pose[i].position.y ;
    ips_yaw = tf::getYaw(msg.pose[i].orientation);

}

void scan_callback(const sensor_msgs::LaserScan& scan)
{
	currentScan = scan;
	//This function is called when a new LaserScan is receive
}

//Callback function for the Position topic (LIVE)
/*
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{

	ips_x X = msg.pose.pose.position.x; // Robot X psotition
	ips_y Y = msg.pose.pose.position.y; // Robot Y psotition
	ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
	ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
}*/

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty 
//	  vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y) {

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;
    
    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s) y0+=sgn(dy2); else x0+=sgn(dx2);
        if (d < 0) d += inc1;
        else {
            d += inc2;
            if (s) x0+=sgn(dx2); else y0+=sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
	ros::Subscriber scan_sub = n.subscribe("/scan", 1, scan_callback);

    //Setup topics to Publish from this node
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    ros::Publisher map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);

	//Occupancy Grid Variable
	nav_msgs::OccupancyGrid occupancyGrid;
	
	/*occupancyGrid.data = new int[50][50];
	// initialize grid
	for(int i = 0; i < 50; i++) {
		for (int j = 0; j < 50; j++)
		occupancyGrid.data[i][j] = 1/2500;		
	}
	*/
    //Set the loop rate
    ros::Rate loop_rate(30);    //20Hz update rate

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages
		// iterate through laser scan
		for (int i = 0; i < scanSize; i++)
		{
			double thetaScan = i*currentScan.angle_increment + currentScan.angle_min + ips_yaw;
			double x = currentScan.ranges[i] * std::cos(thetaScan) + ips_x;
			double y = currentScan.ranges[i] * std::sin(thetaScan) + ips_y;
		}

    	//velocity_publisher.publish(vel); // Publish the command velocity
		map_publisher.publish(occupancyGrid);
    }

    return 0;
}
