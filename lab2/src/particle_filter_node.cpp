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

#include <vector>
#include <cstdlib>
#include <ctime>

void scan_callback(const sensor_msgs::LaserScan& scan)
{
    //This function is called when a new LaserScan is received

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
    ros::Subscriber pose_sub = n.subscribe("/scan", 1, scan_callback);

    //Setup topics to Publish from this node
    ros::Publisher map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    
    //Occupancy Grid Variable
    nav_msgs::OccupancyGrid occupancyGrid;    
        
    //Set the loop rate
    ros::Rate loop_rate(3);    //3Hz update rate

    int M = 8; //Number of particles/samples
    std::vector<float> particle_x (M);
    std::vector<float> particle_y (M);
    std::vector<float> particle_x_new (M);
    std::vector<float> particle_y_new (M);

    std::vector<float> particle_weight (M);
    std::vector<float> cumsum (M);

    /* C++ 11 not a good idea with ros
    //random device and engine initialization
    std::random_device rd;
    std::mt19937 e2(rd());
    */

    //TODO: take a measurement and initialize the particles
    while (ros::ok())
    { 
        loop_rate.sleep(); //Maintain the loop rate
        ros::spinOnce();   //Check for new messages
    
        //Main loop code:
        
        //apply motion model to each sample
        int i;
        for (i = 0; i < M; i++)
        {
            particle_x[i] = i; 
            particle_y[i] = i - 0.5;
        }

        //TODO: get measurement

        //apply measurement model to determine particle weights
        for (i = 0; i < M; i++)
        {
            particle_weight[i] = i; //TODO:measurment model

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

        //map_publisher.publish(occupancyGrid); // Publish the command velocity
    }

    return 0;
}
