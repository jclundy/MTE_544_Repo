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
#define SIMULATION

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_datatypes.h>
//#include <boost/math/distributions/normal.hpp>
#include <random>

#include <vector>
#include <cstdlib>
#include <ctime>

//visualization includes
#include <cmath>
#include <visualization_msgs/Marker.h>

ros::Publisher marker_pub;

double ips_x;
double ips_y;
double ips_yaw;

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
    ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
}
#endif

void motion_model(float * particle_x, float * particle_y, float * theta, geometry_msgs::Twist vel, float d_t, std::default_random_engine * generator)
{
    float prev_x = * particle_x;
    float prev_y = * particle_y;
    float prev_theta = * theta;


    std::normal_distribution<double> distribution(0,0.5);
    float noise_x = distribution(*generator);
    float noise_y = distribution(*generator);

    ROS_INFO("noise_x = [%f]    noise_y = [%f]", noise_x, noise_y);

    float v = vel.linear.x;
    *particle_x = prev_x + v * cos(prev_theta) * d_t + noise_x; 
    *particle_y = prev_y + v* sin(prev_theta) * vel.linear.x * d_t + noise_y;
    *theta = prev_theta + vel.angular.z;
}

float normal_pdf(float x, float m, float s)
{
    static const float inv_sqrt_2pi = 0.3989422804014327;
    float a = (x - m) / s;

    return inv_sqrt_2pi / s * std::exp(-0.5f * a * a);
}

float * cumsum (float * arr)
{
    return arr;
}

int main(int argc, char **argv)
{
    //Initialize the ROS framework
    ros::init(argc,argv,"particle_filter_node");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
#ifdef SIMULATION
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
#else
    ros::Subscriber pose_sub = n.subscribe("/geometry_msgs/PoseWithCovarianceStamped", 1, pose_callback);
#endif

	//Initialize visualization publisher
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        
    //Set the loop rate
    ros::Rate loop_rate(3);    //3Hz update rate

    int M = 8; //Number of particles/samples
    float Q = 1; //Standard deviation of degraded IPS measurement
    std::vector<float> particle_x (M);
    std::vector<float> particle_y (M);
    std::vector<float> particle_x_new (M);
    std::vector<float> particle_y_new (M);

    std::vector<float> particle_weight (M);
    std::vector<float> cumsum (M);

    /* 
    //random device and engine initialization
    std::random_device rd;
    std::mt19937 e2(rd());
    */

    std::default_random_engine generator;
    std::normal_distribution<double> ips_distribution(0,Q);
    //TODO: take a measurement and initialize the particles
    while (ros::ok())
    { 
        loop_rate.sleep(); //Maintain the loop rate
        ros::spinOnce();   //Check for new messages
    
        //Main loop code:
	
	// Marker initialization
	visualization_msgs::Marker points;
    	points.header.frame_id = "/map";
    	points.header.stamp = ros::Time::now();
    	points.ns = "particle_filter_node";
    	points.action = visualization_msgs::Marker::ADD;
    	points.pose.orientation.w = 1.0;
	points.id = 0;
	//points formatting
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.2;
    	points.scale.y = 0.2;
	points.color.g = 1.0f;
    	points.color.a = 1.0;

        //apply motion model to each sample
        int i;
        for (i = 0; i < M; i++)
        {
            float theta = 0; //for testing
            geometry_msgs::Twist vel; //for testing
            vel.linear.x = 0.1; //for testing
            vel.angular.z = 0.3; //for testing
            motion_model (&particle_x[i], &particle_y[i], &theta, vel, 0.01, &generator);
        }

        float y_x = ips_x + ips_distribution(generator);
        float y_y = ips_y + ips_distribution(generator);

        //apply measurement model to determine particle weights
        for (i = 0; i < M; i++)
        {
            particle_weight[i] = normal_pdf(y_x, particle_x[i], Q)
                                    * normal_pdf(y_y, particle_y[i], Q); //TODO:measurment model

            ROS_INFO("Weight[%i] = [%f]", i, particle_weight[i]);

            if (i == 0)
                cumsum[0] = particle_weight[i];
            else
                cumsum [i] = cumsum [i-1] + particle_weight[i];

            ROS_INFO("cumsum[%i] = [%f]", i, cumsum[i]);
        }

        //create a new sample set based on sample weights
        //C++ 11 not compatible!! std::uniform_real_distribution<> uniform_dist(0, cumsum[M-1]);
        for (i = 0; i < M; i++)
        {
            float random_num = rand() * cumsum [M-1] / RAND_MAX;
            //TODO: replace with binary search
            ROS_INFO("RANDOM NUM = [%f]", random_num);
            int j = 0;
            while (random_num > cumsum[j])
            {
                j++;
            }

            //copy the selected particle into the new sample set
            particle_x_new[i] = particle_x[j];
            particle_y_new[i] = particle_y[j];
        }

        for (i = 0; i < M; i++)
        {
            ROS_INFO("NEW SAMPLE SET <%i> [%f][%f]", i, particle_x_new[i], particle_y_new[i]);
        }

        //TODO: check if this is O(n) or O(1), possibly optimize
        particle_x = particle_x_new;
        particle_y = particle_y_new;

        //publish points to rviz
	for (uint32_t i = 0; i < 100; ++i)
	    {
	    
	      geometry_msgs::Point p;
	      p.x = particle_x[i];
	      p.y = particle_y[i];
	      p.z = 0;

	      points.points.push_back(p);

	    }
	marker_pub.publish(points);
    }

    return 0;
}
