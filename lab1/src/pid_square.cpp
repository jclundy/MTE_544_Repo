//  ///////////////////////////////////////////////////////////
//
// jake_square.cpp
// This file contains example code for use with ME 597 lab 1
// It outlines the basic setup of a ros node and the various
// inputs and outputs.
//
// Author: Jacob Rampertab
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

// Const
bool measure_mode = true;

// Output current position
float X;
float Y;
float Yaw;

float abs_float(float a) {
    if(a < 0)
        return a*-1;
    else
        return a;
}

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	//This function is called when a new position message is received
	X = msg->pose.pose.position.x; // Robot X position
	Y = msg->pose.pose.position.y; // Robot X position
 	float temp = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw
    if(temp < 0)
        Yaw = temp+6.28;
    else
        Yaw = temp;
    ROS_INFO("Received: [%f] [%f] [%f]", X, Y, Yaw);
}


int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);

    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    //PID Constants
    int state = 0;
    float target_x = 0;
    float target_y = 1;
    float lin_Kp = 1;
    float ang_Kp = 2;

    bool ang_pid_stop = false;

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

        // Forward PID Controller
        float xdiff = target_x-X;
        float ydiff = target_y-Y;
        float linear_err = sqrt(xdiff*xdiff + ydiff*ydiff);
        vel.linear.x = linear_err * lin_Kp;

        // Angular PID Controller
        float angular_err = 0;
        if(!ang_pid_stop) {
            float target_yaw = 0;
            if(xdiff != 0)
                target_yaw = atan(ydiff/xdiff); //check edge cases
            else
                target_yaw = 1.57;

            if(target_y<0)
                target_yaw += 3.14;
            angular_err = target_yaw-Yaw;
            if(fabs(angular_err) < 0.1)
                ang_pid_stop = true;
            vel.angular.z = angular_err * ang_Kp;
            ROS_INFO("ANGPID TY[%f] AE[%f] P[%f]", target_yaw, angular_err, vel.angular.z);
        } else {
            vel.angular.z = 0;
        }

        // State Controller
        //ROS_INFO("Error [%f] [%f] [%f]", xdiff, ydiff, angular_err);
        if(fabs(linear_err) < 0.2 && fabs(angular_err) < 0.1){
            ROS_INFO("State Transition");
            ang_pid_stop = false;
            switch(state) {
                case 0:
                    target_x = 1;
                    target_y = 1;
                    state = 1;
                    break;
                case 1:
                    target_x = 1;
                    target_y = 0;
                    state = 2;
                    break;
                case 2:
                    target_x = 0;
                    target_y = 0;
                    state = 3;
                    break;
                case 3:
                    target_x = 0;
                    target_y = 1;
                    state = 0;
                    break;
                default:
                    ROS_INFO("ERROR");
                    break;
            }
        }

        velocity_publisher.publish(vel);
    }

    return 0;
}
