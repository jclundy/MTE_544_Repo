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
#define VERBOSE_DEBUG

#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
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


double ips_x = 0;
double ips_y = 0;
double ips_yaw;
double forward_v;
double omega;
float motion_model_std = 0.05;

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

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
    forward_v = odom->twist.twist.linear.x;
    omega = odom->twist.twist.angular.z;
    ROS_DEBUG("odom_callback v: %f omega: %f", forward_v, omega);
}

void motion_model(double * particle_x, double * particle_y, double * theta, double d_t, std::default_random_engine * generator)
{
    double prev_x = * particle_x;
    double prev_y = * particle_y;
    double prev_theta = * theta;


    std::normal_distribution<double> distribution(0,motion_model_std);
    double noise_x = distribution(*generator);
    double noise_y = distribution(*generator);

   // ROS_INFO("noise_x = [%f]    noise_y = [%f]", noise_x, noise_y);

    double v = forward_v;
    *particle_x = prev_x + v * cos(prev_theta) * d_t + noise_x; 
    *particle_y = prev_y + v * sin(prev_theta) * d_t + noise_y;
    *theta = prev_theta + omega;
}

double normal_pdf(double x, double m, double s)
{
    static const double inv_sqrt_2pi = 0.3989422804014327;
    double a = (x - m) / s;

    return inv_sqrt_2pi / s * std::exp(-0.5f * a * a);
}

double * cumsum (double * arr)
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
    ros::Subscriber odom_sub = n.subscribe("/odom", 1, odom_callback);

	//Initialize visualization publisher
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        
    //Set the loop rate
    float freq = 10;
    float d_t = 1/freq;
    ros::Rate loop_rate(freq);    //3Hz update rate

    int M = 200; //Number of particles/samples
    float Q = 0.1; //Standard deviation of degraded IPS measurement
    float stop_threshold = 0.01; //the speed under which the robot is considered stopped
    std::vector<double> particle_x (M);
    std::vector<double> particle_y (M);
    std::vector<double> particle_x_new (M);
    std::vector<double> particle_y_new (M);

    std::vector<double> particle_weight (M);
    std::vector<double> cumsum (M);

    /* 
    //random device and engine initialization
    std::random_device rd;
    std::mt19937 e2(rd());
    */

    std::default_random_engine generator;
    std::normal_distribution<double> ips_distribution(0,Q);
    //take a measurement and initialize the particles
    while (ips_x == 0 && ips_y == 0){ros::spinOnce();}
    for (int i = 0; i < M; i++)
    {
        particle_x[i] = ips_x;
        particle_y[i] = ips_y;
    }
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
        points.scale.x = 0.05;
        points.scale.y = 0.05;
        points.color.g = 1.0f;
        points.color.a = 1.0;

        ROS_INFO("odom_callback v: %f omega: %f", forward_v, omega);
        if(forward_v > stop_threshold || forward_v < -stop_threshold)
        {
            //apply motion model to each sample
            int i;
            for (i = 0; i < M; i++)
            {
                motion_model (&particle_x[i], &particle_y[i], &ips_yaw, d_t, &generator);
            }

            double y_x = ips_x + ips_distribution(generator);
            double y_y = ips_y + ips_distribution(generator);

            //apply measurement model to determine particle weights
            for (i = 0; i < M; i++)
            {
                particle_weight[i] = normal_pdf(y_x, particle_x[i], Q)
                                        * normal_pdf(y_y, particle_y[i], Q);

                //ROS_INFO("Weight[%i] = [%f]", i, particle_weight[i]);

                if (i == 0)
                    cumsum[0] = particle_weight[i];
                else
                    cumsum [i] = cumsum [i-1] + particle_weight[i];

                //ROS_INFO("cumsum[%i] = [%f]", i, cumsum[i]);
            }

            //create a new sample set based on sample weights
            //C++ 11 not compatible!! std::uniform_real_distribution<> uniform_dist(0, cumsum[M-1]);
            for (i = 0; i < M; i++)
            {
                double random_num = rand() * cumsum [M-1] / RAND_MAX;
                //TODO: replace with binary search
                //ROS_INFO("RANDOM NUM = [%f]", random_num);
                int j = 0;
                while (random_num > cumsum[j])
                {
                    j++;
                }

                //copy the selected particle into the new sample set
                particle_x_new[i] = particle_x[j];
                particle_y_new[i] = particle_y[j];
            }
    /*
            for (i = 0; i < M; i++)
            {
                ROS_INFO("NEW SAMPLE SET <%i> [%f][%f]", i, particle_x_new[i], particle_y_new[i]);
            }
    */
            //TODO: check if this is O(n) or O(1), possibly optimize
            particle_x = particle_x_new;
            particle_y = particle_y_new;
        }
        //publish points to rviz
	    for (uint32_t i = 0; i < M; ++i)
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
