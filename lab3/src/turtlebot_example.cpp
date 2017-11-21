//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 3
//
// Author: James Servos
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>
#include "Node.h"
#include <vector>
#include <cstdlib>
#include "Graph.h"
#include <ctime>

ros::Publisher marker_pub;
#define GRID_SIZE 100
#define NUM_SAMPLES 10
#define TAGID 0
#define PI 3.14159265
#define SIMULATION

float ips_x = 0;
float ips_y = 0;
float ips_yaw;
float forward_v;
float omega;

double occ_grid[GRID_SIZE][GRID_SIZE];
Graph graph;

struct Temp_node
{
  double x;
  double y;
};

#ifdef SIMULATION
//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg)
{
    int i;
    for(i = 0; i < msg.name.size(); i++) if(msg.name[i] == "mobile_base") break;

    ips_x = msg.pose[i].position.x ;
    ips_y = msg.pose[i].position.y ;
    ips_yaw = tf::getYaw(msg.pose[i].orientation);
}

#else
//Callback function for the Position topic (LIVE)

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{

    ips_x X = msg.pose.pose.position.x; // Robot X psotition
    ips_y Y = msg.pose.pose.position.y; // Robot Y psotition
    ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
    //ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
}
#endif

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
    forward_v = odom->twist.twist.linear.x;
    omega = odom->twist.twist.angular.z;
    //ROS_DEBUG("odom_callback v: %f omega: %f", forward_v, omega);
}


//Example of drawing a curve
void drawCurve(int k)
{
   // Curves are drawn as a series of stright lines
   // Simply sample your curves into a series of points

   double x = 0;
   double y = 0;
   double steps = 50;

   visualization_msgs::Marker lines;
   lines.header.frame_id = "/map";
   lines.id = k; //each curve must have a unique id or you will overwrite an old ones
   lines.type = visualization_msgs::Marker::LINE_STRIP;
   lines.action = visualization_msgs::Marker::ADD;
   lines.ns = "curves";
   lines.scale.x = 0.1;
   lines.color.r = 1.0;
   lines.color.b = 0.2*k;
   lines.color.a = 1.0;

   //generate curve points
   for(int i = 0; i < steps; i++) {
       geometry_msgs::Point p;
       p.x = x;
       p.y = y;
       p.z = 0; //not used
       lines.points.push_back(p);

       //curve model
       x = x+0.1;
       y = sin(0.1*i*k);
   }

   //publish new curve
   marker_pub.publish(lines);

}

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
    // Assuming 100x100 map input, will complain if that doesn't match
    if(msg.info.width != GRID_SIZE || msg.info.height != GRID_SIZE) {
        ROS_INFO("Inconsistent map sizes, dumping...");
        return;
    }

    // Marker initialization
    visualization_msgs::Marker points;
    points.header.frame_id ="/map";
    points.header.stamp = ros::Time::now();
    points.ns = "node_samples";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.z = -0.7071; //to match amcl map
    points.pose.orientation.w = 0.7071;
    points.pose.position.x = -1;
    points.pose.position.y = 5;
    points.id = 0;
    //points formatting
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.05;
    points.scale.y = 0.05;
    points.color.r = 1.0;
    points.color.a = 1.0;

    // Reformat input map
    for(int i = 0; i < GRID_SIZE*GRID_SIZE; i++) {
        occ_grid[GRID_SIZE-1 - i/GRID_SIZE][i%GRID_SIZE] = msg.data[i];
    }

    // Random node placement
    srand(time(NULL));
    for(int j = 0; j < NUM_SAMPLES; j) {
        int x = rand()%GRID_SIZE;
        int y = rand()%GRID_SIZE;
        if(occ_grid[x][y] == 0 && graph.add_new_node(x, y)) {
            geometry_msgs::Point p;
            p.x = x*0.1;
            p.y = y*0.1;
            p.z = 0;
            points.points.push_back(p);
            j++;
        }
    }

    // Publish sampling nodes to RVIZ
    marker_pub.publish(points);
}

float set_speed(float target_x, float target_y, float prev_theta_error, ros::Publisher velocity_publisher)
{
  float top_speed = 1;
  float k = 1;
  float kp = 0.5;
  //Velocity control variable
  float theta_ref = atan2(target_y - ips_y, target_x - ips_x);
  float theta_error = theta_ref - ips_yaw;

  if (theta_error > PI)
  {
    theta_error -= 2*PI;
  }
  else if (theta_error < -PI)
  {
    theta_error += 2*PI;
  }

  geometry_msgs::Twist vel;

  float cos_error = cos (theta_error);
  float forward_v = cos_error * cos_error * cos_error * top_speed;
  float omega = k * theta_error + kp * (theta_error - prev_theta_error);

  vel.linear.x = forward_v; // set linear speed
  vel.angular.z = omega; // set angular speed

  velocity_publisher.publish(vel); // Publish the command velocity
  return theta_error;
}

float get_error_magnitude(float target_x, float target_y)
{
  float diff_x = target_x - ips_x;
  float diff_y = target_y - ips_y;
  return sqrt(diff_x * diff_x + diff_y * diff_y);
}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;
    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);

    //Subscribe to the desired topics and assign callbacks
#ifdef SIMULATION
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
#else
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
#endif
    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate
    float theta_error = 0;

    uint num_waypoints = 5;
    std::vector<Node> waypoints;
    waypoints.push_back(Node(0,0,0));
    waypoints.push_back(Node(1,10,10));
    waypoints.push_back(Node(2,10,-10));
    waypoints.push_back(Node(3,-10,-10));
    waypoints.push_back(Node(4,-10,10));
    /*
    uint num_waypoints = 5;
    std::vector<Temp_node> waypoints;
    for (int i = 0; i < num_waypoints; i++)
    {
      waypoints.push_back(Temp_node());
    }
    waypoints[0].x = 0;
    waypoints[0].y = 0;
    waypoints[1].x = 10;
    waypoints[1].y = 10;
    waypoints[2].x = 10;
    waypoints[2].y = -10;
    waypoints[3].y = -10;
    waypoints[3].x = -10;
    waypoints[4].y = -10;
    waypoints[4].y = 10;
    */

/*
    std::vector<Node> waypoints;
    waypoints.push_back(Node(0,0,0));
    waypoints.push_back(Node(1,10,-10));
    waypoints.push_back(Node(2,10,10));
    waypoints.push_back(Node(3,-10,10));
    waypoints.push_back(Node(4,-10,-10));
*/
     uint wpt_ind = 0;
     bool graph_generated = false;
     bool graph_drawn = false;

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

      if(!graph_generated && graph.nodeList.size() == NUM_SAMPLES)
      {
        graph_generated = true;
      }

      if(!graph_drawn && graph_generated)
      {
        ROS_INFO("before generating edges \n");
        graph.print_graph_to_console();
        ROS_INFO("after generating edges \n");
        graph.generate_connections(0, 100);
        graph.print_graph_to_console();
        graph.draw_in_rviz(marker_pub);
        graph_drawn = true;
      }


      graph.draw_in_rviz(marker_pub);

      if (wpt_ind >= num_waypoints)
      {
        wpt_ind = 0;
      }
      float error_mag = get_error_magnitude(waypoints[wpt_ind].x, waypoints[wpt_ind].y);
      theta_error = set_speed(waypoints[wpt_ind].x, waypoints[wpt_ind].y, theta_error, velocity_publisher);
      //ROS_INFO("theta_error = %f,   x,y = %f \t %f    dist=%f", theta_error, ips_x, ips_y, error_mag);

      if (error_mag < 0.05)
      {
        wpt_ind++;
      }
    }

    return 0;
}
