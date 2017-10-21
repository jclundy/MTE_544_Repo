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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <math.h>

double X;
double Y;
double Yaw;
const double squareLength = 1;
const double squareAngle = 1.57; // pi/2 radians

//Callback function for the Position topic 
//void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
//{
	//This function is called when a new position message is received

//	double X = msg->pose.pose.position.x; // Robot X psotition
//	double Y = msg->pose.pose.position.y; // Robot Y psotition
//	double Yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw

//}

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	//This function is called when a new position message is received

	X = msg->pose.pose.position.x; // Robot X psotition
	Y = msg->pose.pose.position.y; // Robot Y psotition
 	Yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw

}

double displacement(double x1, double x2, double y1, double y2) {
	double dx = x2 - x1;
	double dy = y2 - y1;
	return std::sqrt(std::pow(dx,2) + std::pow(dy,2));
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
	bool isTurning = true;
	bool isStopped = true;
	int timerCount = 0;
	double turnRate = 1;
	double linearRate = 1;
	double desiredDistance = 5;
	double currentDistance = 0;
	double referenceX = X;
	double referenceY = Y;
	double oldYaw = Yaw;
	
    while (ros::ok())
    {
		
		double angleDiff = std::abs(Yaw - oldYaw);
		double currentX = X;
		double currentY = Y;
		

		ROS_INFO("Current X - [%f] , Y - [%f], Yaw - [%f]",X,Y,Yaw);
		// first we check if the robot is in a stopped state
		// it should stop for 1 second after completing a turn, or traversing one side of the square	
		if (isStopped) {
			ROS_INFO("Stopped");
			timerCount ++;
			// stops for 1 second (1/20th s * 20)
			if(timerCount > 20) { 
				timerCount = 0;
				// after setting isStopped to false, the robot should be able to turn / drive forward based on the 'isTurning' flag
				isStopped = false;
				// after the robot has stopped for 1s, we update the reference position values
				// these reference position values are used to as a reference point for how far it will turn / drive forward
				referenceX = X;
				referenceY = Y;
				oldYaw = Yaw;
			}
			vel.angular.z = 0;
			vel.linear.x = 0;
		}
		// next we check if the robot is in a turning state
		// it should check its position reading and keep turning until its 		
		else if(isTurning) {
			ROS_INFO("Turning");
			ROS_INFO("Angle difference : [%f]", angleDiff);
			// stop turning at 80% of target angle to so robot doesn't overturn do to inertio			
			if(angleDiff < squareAngle * 0.9) { 
		   		vel.angular.z = 0.5;
		   		vel.linear.x = 0;
			} else {
				ROS_INFO("Done turning");
				vel.angular.z = 0;
		   		vel.linear.x = 0;
				// when done turning, we set the turning state to false, and enter a 'stopped' state
				isTurning = false;
				isStopped = true;
			}
		} else {
			ROS_INFO("Driving Forward");
			ROS_INFO("referenceX - [%f], referenceY - [%f], currentX - [%f], currentY - [%f]",referenceX, currentX, referenceY, currentY);
			ROS_INFO("currentDistance - [%f]", currentDistance);
			// displacement is measured using the trigonometric distance from its reference (x,y) point			
			currentDistance = displacement(referenceX, currentX, referenceY, currentY);
			ROS_INFO("currentDistance - [%f]", currentDistance);			
			if(currentDistance < squareLength) {
				vel.linear.x = 0.5;
				vel.angular.z = 0;
			} else {
				ROS_INFO("Done Driving Forward");
				vel.linear.x = 0;
				vel.angular.z = 0;
				//when done turning, we set the turning state to true, and the 'stopped' state to true as well
				//the robot will first stop for 1 s, when it is done in its stopped state it will then execute its turn.
				isTurning = true;
				isStopped = true;
			}
			
		}
		// here is where we actually send the velocity commands 
		velocity_publisher.publish(vel);
			
		loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages
    }

    return 0;
}
