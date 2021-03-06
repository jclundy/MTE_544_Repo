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
#include <queue>
#include <visualization_msgs/Marker.h>

ros::Publisher marker_pub;


double ips_x = 0;
double ips_y = 0;
double ips_yaw;
double forward_v;
double omega;
float motion_model_std = 0.01;
double y_x = 0;
double y_y = 0;

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

double calculate_state_estimate_error (std::vector<double>* particle_x,std::vector<double>* particle_y, int  M){
    double x_total = 0;
    double y_total = 0;
    for (int i = 0; i < M; i++){
        x_total += (*particle_x)[i];
        y_total += (*particle_y)[i];
    }
    double x_error = ips_x - x_total / M;
    double y_error = ips_y - y_total / M;
    return sqrt(x_error * x_error + y_error * y_error);
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

    int M = 1000; //Number of particles/samples
    float Q = 0.1; //Standard deviation of degraded IPS measurement
    float stop_threshold = 0.01; //the speed under which the robot is considered stopped
    std::vector<double> particle_x (M);
    std::vector<double> particle_y (M);
    std::vector<double> particle_x_new (M);
    std::vector<double> particle_y_new (M);

    std::vector<double> particle_weight (M);
    std::vector<double> cumsum (M);


    uint error_queue_capacity = 50;
    std::queue<double> error_queue;
    double average_error;
    /* 
    //random device and engine initialization
    std::random_device rd;
    std::mt19937 e2(rd());
    */

    std::default_random_engine generator;
    std::normal_distribution<double> ips_distribution(0,Q);
    std::normal_distribution<double> init_distribution(0,5);
    //take a measurement and initialize the particles

    for (int i = 0; i < M; i++)
    {
        particle_x[i] = init_distribution(generator);
        particle_y[i] = init_distribution(generator);
    }

    average_error = calculate_state_estimate_error(&particle_x, &particle_y, M);

    for (int i = 0; i < error_queue_capacity; i++)
    {
        error_queue.push(average_error);
    }

    while (ros::ok())
    { 
        loop_rate.sleep(); //Maintain the loop rate
        ros::spinOnce();   //Check for new messages
    
        //Main loop code:
    
        // Marker initialization
        visualization_msgs::Marker points, true_ips_point, noisy_ips_point;
        points.header.frame_id = true_ips_point.header.frame_id = noisy_ips_point.header.frame_id = "/map";
        points.header.stamp = true_ips_point.header.stamp = noisy_ips_point.header.stamp = ros::Time::now();
        points.ns = true_ips_point.ns = noisy_ips_point.ns = "particle_filter_node";
        points.action = true_ips_point.action = noisy_ips_point.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = true_ips_point.pose.orientation.w = noisy_ips_point.pose.orientation.w = 1.0;
        points.id = 0;
        true_ips_point.id = 1;
        noisy_ips_point.id = 2; 
        //points formatting
        points.type = true_ips_point.type = noisy_ips_point.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.06;
        points.scale.y = 0.06;
        true_ips_point.scale.x = 0.2;
        true_ips_point.scale.y = 0.2;
        noisy_ips_point.scale.x = 0.2;
        noisy_ips_point.scale.y = 0.2;
        points.color.b = 1.0;
        points.color.a = 1.0;
        true_ips_point.color.g = 1.0f;
        true_ips_point.color.a = 1.0;
        noisy_ips_point.color.r = 1.0;
        noisy_ips_point.color.a = 1.0;

        //ROS_INFO("odom_callback v: %f omega: %f", forward_v, omega);
        if(forward_v > stop_threshold || forward_v < -stop_threshold)
        {
            //apply motion model to each sample
            int i;
            for (i = 0; i < M; i++)
            {
                motion_model (&particle_x[i], &particle_y[i], &ips_yaw, d_t, &generator);
            }

            y_x = ips_x + ips_distribution(generator);
            y_y = ips_y + ips_distribution(generator);

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

            //TODO: check if this is O(n) or O(1), possibly optimize
            particle_x = particle_x_new;
            particle_y = particle_y_new;

            double err = calculate_state_estimate_error(&particle_x, &particle_y, M);
            average_error += err / error_queue_capacity;
            average_error -= error_queue.front() / error_queue_capacity;
            error_queue.pop();
            error_queue.push(err);
            ROS_INFO("ERROR in meters [%f, front:%f] : %f  50th:%f",err,error_queue.front(), average_error, average_error/error_queue_capacity);
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
        geometry_msgs::Point true_p;
        true_p.x = ips_x;
        true_p.y = ips_y;
        true_p.z = 0;

        true_ips_point.points.push_back(true_p);
        marker_pub.publish(true_ips_point);

        geometry_msgs::Point noisy_p;
        noisy_p.x = y_x;
        noisy_p.y = y_y;
        noisy_p.z = 0;

        noisy_ips_point.points.push_back(noisy_p);
        marker_pub.publish(noisy_ips_point);
    }

    return 0;
}
