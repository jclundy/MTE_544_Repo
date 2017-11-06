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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

#define DEBUG_INFO
#undef DEBUG_INFO

#define SIM
//#undef SIM

#define GRID_SIZE 100
const double NUMBER_TILES = GRID_SIZE * GRID_SIZE;
const double MAP_UPPER_LIMIT = 7; //meters
const double MAP_LOWER_LIMIT = -7; //meters
const double SCALE = GRID_SIZE / (MAP_UPPER_LIMIT - MAP_LOWER_LIMIT);
const int scanSize = 134;
const int downsample_factor = 5;

ros::Publisher pose_publisher;
ros::Publisher marker_pub;
double current_scan_vector[134];
sensor_msgs::LaserScan current_scan;
geometry_msgs::Pose current_pose;

double ips_x;
double ips_y;
double ips_yaw;

short sgn(int x) { return x >= 0 ? 1 : -1; }

#ifdef SIM
//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg)
{

    int i;
    for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

    ips_x = msg.pose[i].position.x ;
    ips_y = msg.pose[i].position.y ;
    ips_yaw = tf::getYaw(msg.pose[i].orientation);
    current_pose = msg.pose[i];

}

#else
//Callback function for the Position topic (LIVE)
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{

	ips_x = msg.pose.pose.position.x; // Robot X psotition
	ips_y = msg.pose.pose.position.y; // Robot Y psotition
	ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

    // Set origin for map to 0,0
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(ips_x, ips_y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, ips_yaw);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}
#endif

void scan_callback(const sensor_msgs::LaserScan& scan)
{
	current_scan = scan;
	//This function is called when a new LaserScan is receive
}


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

int convert_coordinate_to_index(double coordinate)
{
	int index = int((coordinate - MAP_LOWER_LIMIT)*SCALE);
	if(index < 0) {
		index = 0;
	}
	if(index >= GRID_SIZE) {
		index = GRID_SIZE - 1;
	}
	return index;
}

void print_occupancy_grid(double og[][GRID_SIZE], int max_x, int max_y, int robot_x, int robot_y)
{
	for(int i = 0; i < max_x; i++)
	{
  		std::string row = "";
		for(int j = 0; j < max_y; j++)
		{
			double prob = og[i][j];
			int val = int(prob);
			int abs_val = abs(val);
			if(robot_x == i && robot_y == j) {
				std::cout << "[ X ]";
			} else {
				std::cout << '['<< val << " ";
				if(abs_val < 100) std::cout << " ";
				if(abs_val < 10) std::cout << " ";
				std::cout << ']';
			}
		}
		std::cout << '\n';
	}
}
int8_t convert_logit_to_prob(double logodds)
{
	double exp = std::exp(logodds);
	return int8_t(exp/(1+exp)*100);
}

int main(int argc, char **argv)
{
	int count = 0;

	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
#ifdef SIM
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
#else
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
#endif
    ros::Subscriber scan_sub = n.subscribe("/scan", 1, scan_callback);

    //Setup topics to Publish from this node
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    ros::Publisher map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    // Initialize velocity
    geometry_msgs::Twist vel;
	//Initialize Occupancy Grid
	double occupancy_grid[GRID_SIZE][GRID_SIZE];
	nav_msgs::OccupancyGrid occupancy_grid_message;
	occupancy_grid_message.data.reserve(NUMBER_TILES);
	occupancy_grid_message.info.resolution = 1/SCALE; // 0.2 meters per cell
	occupancy_grid_message.info.width = GRID_SIZE;
	occupancy_grid_message.info.height = GRID_SIZE;
	occupancy_grid_message.info.origin.position.x = MAP_LOWER_LIMIT;
	occupancy_grid_message.info.origin.position.y = MAP_LOWER_LIMIT;

	double initial_odds = 0.2;
	double initial_log_odds = std::log(initial_odds/ (1 - initial_odds));

	for(int i = 0; i < GRID_SIZE; i++) {
		for (int j = 0; j < GRID_SIZE; j++) {
			occupancy_grid[i][j] = initial_log_odds;
		}
	}
	for (int i = 0; i < NUMBER_TILES; i++) {
		occupancy_grid_message.data.push_back(-1);
	}
    //Set the loop rate in Hz
    ros::Rate loop_rate(20);

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages
		// iterate through laser scan
		int x0 = convert_coordinate_to_index(ips_x);
		int y0 = convert_coordinate_to_index(ips_y);
		if(count < 3) {
			count++;
			continue;
		}
		int x1 = 0;
		int y1 = 0;
		for (int i = 0; i < scanSize; i+= downsample_factor)
		{
			double reading = current_scan.ranges[i];
			if(std::isnormal(reading)) {
				// determine real world coordinates of laser scan endpoint
				double theta = i*current_scan.angle_increment + current_scan.angle_min + ips_yaw;
				double x = current_scan.ranges[i] * std::cos(theta) + ips_x;
				double y = current_scan.ranges[i] * std::sin(theta) + ips_y;

				// convert laser endpoint to grid index
				if(abs(x) > MAP_UPPER_LIMIT || abs(y) > MAP_UPPER_LIMIT)
					continue;
				x1 = convert_coordinate_to_index(x);
				y1 = convert_coordinate_to_index(y);
				// convert current position to grid index

				// determine indices of OG tiles traversed by laser beam
				std::vector<int> x_tile_indices;
				std::vector<int> y_tile_indices;
				bresenham(x0, y0, x1, y1, x_tile_indices, y_tile_indices);

				// update probability values for each tile crossed laser ray
				double occupied = 0.9;
				double empty = 0.1;
				int list_length = x_tile_indices.size();
				for(int i = 0; i < list_length; i++)
				{
					int x_index = x_tile_indices[i];
					int y_index = y_tile_indices[i];
					occupancy_grid[x_index][y_index] += std::log(empty/(1-empty)) - initial_log_odds;
					int grid_message_index = x_index + GRID_SIZE * y_index;
					occupancy_grid_message.data[grid_message_index] = convert_logit_to_prob(occupancy_grid[x_index][y_index]);
				}
				// update probability value for laser ray endpoint
				occupancy_grid[x1][y1] += std::log(occupied/(1-occupied))- initial_log_odds;
				occupancy_grid_message.data[x1 + GRID_SIZE * y1] = convert_logit_to_prob(occupancy_grid[x1][y1]);
			}
		}
		//update map
		map_publisher.publish(occupancy_grid_message);
		ROS_INFO("robot position [%f, %f] ", ips_x, ips_y);
		ROS_INFO("robot grid position [%d, %d] ", x0, y0);
		ROS_INFO("Updated occupancy grid");
		#ifdef DEBUG_INFO
		print_occupancy_grid(occupancy_grid, GRID_SIZE, GRID_SIZE, x0, y0);
		#endif
		//update velocity
		vel.angular.z = 0.5;
		velocity_publisher.publish(vel);
    }

    return 0;
}
