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
#include "Edge.h"
#include <vector>
#include <cstdlib>
#include "Graph.h"
#include <ctime>

#include "rviz_draw.h"

ros::Publisher marker_pub;
RViz_Draw drawer;
#define GRID_SIZE 100
#define NUM_SAMPLES 100
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

   //visualization_msgs::Marker lines;
   //lines.header.frame_id = "/map";
   //lines.id = k; //each curve must have a unique id or you will overwrite an old ones
   //lines.type = visualization_msgs::Marker::LINE_STRIP;
   //lines.action = visualization_msgs::Marker::ADD;
   //lines.ns = "curves";
   //lines.scale.x = 0.1;
   //lines.color.r = 1.0;
   //lines.color.b = 0.2*k;
   //lines.color.a = 1.0;

   drawer.claim(visualization_msgs::Marker::LINE_STRIP);

   //generate curve points
   for(int i = 0; i < steps; i++) {
       //geometry_msgs::Point p;
       //p.x = x;
       //p.y = y;
       //p.z = 0; //not used
       //lines.points.push_back(p);
       drawer.add_point(x, y);

       //curve model
       x = x+0.1;
       y = sin(0.1*i*k);
   }

   //publish new curve
   //marker_pub.publish(lines);
   drawer.pub();
   drawer.release();
}
void generate_graph(const nav_msgs::OccupancyGrid& msg, ros::Publisher publisher)
{
  //ROS_INFO("before generating edges \n");
  //graph.print_graph_to_console();
  //ROS_INFO("after generating edges \n");
  graph.generate_connections(0, 40);
  //graph.print_graph_to_console();
  //std::string np1 = "graph1";
  //std::string np1 = "graph2";
  graph.draw_in_rviz(publisher,10, 0, 1, 0, 1, "node_samples");
  ROS_INFO("before pruning edges \n");
  graph.print_graph_to_console();
  graph.prune_invalid_connections(msg, 0.3, 0);
  ROS_INFO("after pruning edges \n");
  graph.print_graph_to_console();
  graph.draw_in_rviz(publisher,11, 0, 0, 1, 1, "node_samples");
}
//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
    // Assuming 100x100 map input, will complain if that doesn't match
    if(msg.info.width != GRID_SIZE || msg.info.height != GRID_SIZE) {
        ROS_INFO("actual width %d, height %d, vs GRID_SIZE %i", msg.info.width, msg.info.height, GRID_SIZE);
        ROS_INFO("Inconsistent map sizes, dumping...");
        return;
    }

    drawer.pub();
    drawer.release();

    // Publish sampling nodes to RVIZ
    generate_graph(msg, marker_pub);
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

//calculates the distance between 2 nodes. So random RAWR XD
double distance(Node* a, Node* b)
{
    double x_d = a->x - b->x;
    double y_d = a->y - b->y;
    return sqrt((x_d * x_d) + (y_d * y_d));
}

void display_openset_closedset(std::vector<Node*>& open_set, std::vector<Node*>& closed_set)
{
    std::string output_str =" ";

    std::stringstream ss;
    ss << "Closed{";
    for(int i = 0; i < closed_set.size(); i++){
        ss << closed_set[i]->index;
        if (i < closed_set.size()-1){
            ss << ",";
        }
    }

    ss << "}   Open{";
    for(int i = 0; i < open_set.size(); i++){
        ss << open_set[i]->index;
        if (i < open_set.size()-1){
            ss << ",";
        }
    }
    ss << "}";

    output_str += ss.str();

    const char *cstr = output_str.c_str(); 
    ROS_INFO("%s",cstr);
}

void astar(std::vector<Node*>& nodes, std::vector<Node*>& spath, int start_index, int end_index){
    ROS_INFO("Beginning A*");
    ROS_INFO("start node index: %d", nodes[start_index]->index);
    ROS_INFO("End node index: %d", nodes[end_index]->index);
    ROS_INFO("start node x: %f", nodes[end_index]->x);
    //declarations
    std::vector<Node*> open_set;
    std::vector<Node*> closed_set;

    int graph_size = nodes.size(); 
    bool done = 0;

    double dmax = 0;
    double dtogo = 0;
    double dcur = 0; 
    //find max distance as set as lower bound 
    dmax =  distance(nodes[start_index], nodes[end_index]);
    Node* start_node = nodes[start_index];
    start_node->back_pointer_index = -1;
    start_node->lower_bound_cost = dmax;
    start_node->current_cost = 0;
    
    //starting node in open set
    open_set.push_back(start_node);


    Node* best_node = start_node;
    ROS_INFO("Starting While loop");
    while(!done){
        display_openset_closedset(open_set, closed_set);
        //code for if open set is empty, return an empty closed set 

        //if open set not empty
        //iterate through array to find best node and put into closed set
        double min = open_set[0]->lower_bound_cost;
        uint mindex = 0;

        for(int i = 1; i < open_set.size(); i++){
            if (open_set[i]->lower_bound_cost < min)
            {
                min = open_set[i]->lower_bound_cost;
                mindex = i;
            }
        }

        //define the best node in open set
        best_node = open_set[mindex];
        //move best_node to closed set
        closed_set.push_back(best_node);

        //check if best_node is the end_node
        if(best_node->index == end_index){
            done = 1; 
            //move on to next iteration of the loop 
            continue; 
        }

        display_openset_closedset(open_set, closed_set);

        //if edgelist is empty, next iteration loop, iterate again
        if(best_node->edgeList.size()==0)
        {
            ROS_INFO("no edges");
        }
        else
        { 

            ROS_INFO("Starting neighbour loop");
            //process each neighbour 
            for(int j = 0; j < best_node->edgeList.size(); j++){
                Node* neighbour = nodes[best_node->edgeList[j].endNodeIndex];

                ROS_INFO("checking if node is in closed set");
                //check if node is in closed set
                bool found = 0;
                int k = 0;
                for(int k = 0; k < closed_set.size() && !found; k++){
                    if(closed_set[k]->index == neighbour->index){
                        found = 1;
                    }
                }
                if (found)
                {
                    ROS_INFO("node found in closed set, continuing");
                    continue;
                }else{
                    ROS_INFO("node not found in closed set");
                }

                //distance to go
                dtogo = distance(nodes[end_index], neighbour);

                //current distance
                dcur = best_node->current_cost + best_node->edgeList[j].cost;
                ROS_INFO("check if node is in open set and the current distance is lower than previous one");
                //check if node is in open set and the current distance is lower than previous one
                for(int m = 0; m < open_set.size() && !found; m++){
                    if(open_set[k]->index == neighbour->index){
                        found = 1; 
                        if(dcur < open_set[k]->current_cost){
                            open_set[k]->back_pointer_index = best_node->index;
                            open_set[k]->lower_bound_cost = dtogo + dcur;
                            open_set[k]->current_cost = dcur;
                        }
                    }
                }
                if(!found){
                    ROS_INFO("was not found, add to open set");
                    //add endNodeIndex to openSet
                    for(int i=0;i<nodes.size();i++){
                        if (neighbour->index == nodes[i]->index){
                            neighbour->back_pointer_index = best_node->index;
                            neighbour->lower_bound_cost = dtogo + dcur;
                            neighbour->current_cost = dcur;
                            open_set.push_back(neighbour);
                        }    
                    }
                }
            }

            ROS_INFO("done neighbour loop. Mindex:%d", mindex);
        }
        //take best_node out of openset
        open_set[mindex] = open_set.back();
        open_set.pop_back(); 
    }
    ROS_INFO("Done iterating through graph, finding waypoints");

    Node* temp = best_node;
    while (temp->back_pointer_index != -1)
    {
      spath.push_back(temp);
      temp = nodes[temp->back_pointer_index];
    }

    ROS_INFO("Finished A* Path:");
    for (int i = 0; i < spath.size(); i++)
    {
        ROS_INFO("index %d", spath[i]->index);
    }
}



int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    Node graphNode(0,0,0);
    ROS_INFO("defined a new node index : %d x: %f y: %f", graphNode.index, graphNode.x, graphNode.y);
    drawer = RViz_Draw(n); //Initialize drawer for rviz
    ROS_INFO("----Starting----");

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
    
    ROS_INFO("----1----");

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate
    /*
    float theta_error = 0;

    uint num_waypoints = 5;
    std::vector<Node> waypoints;
    waypoints.push_back(Node(0,0,0));
    waypoints.push_back(Node(1,10,10));
    waypoints.push_back(Node(2,10,-10));
    waypoints.push_back(Node(3,-10,-10));
    waypoints.push_back(Node(4,-10,10));
    */
    ROS_INFO("----2----");
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

    loop_rate.sleep();
    ros::spinOnce();  

    /*
    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

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
    */
    ROS_INFO("----3----");

    Node node0(0, 3, 4); 
    Node node1(1, 6, 8); 
    Node node2(2, 15, 15); 
    Node node3(3, 16, 16); 
    Node node4(4, 13, 12); 
    Node node5(5, 2, 1); 

    Edge edge0(distance(&node0,&node1),1); 
    Edge edge1(distance(&node1,&node2),2);
    Edge edge2(distance(&node2,&node3),3);  
    Edge edge3(distance(&node2,&node4),4); 
    Edge edge4(distance(&node3,&node5),5); 

    //node1.edgeList.push_back(edge1);
    node0.addEdge(edge0); 
    node1.addEdge(edge1); 
    node2.addEdge(edge2); 
    node2.addEdge(edge3); 
    node3.addEdge(edge4); 

    ROS_INFO("----4----%i", node1.edgeList[0].endNodeIndex);
    ROS_INFO("----4----%i", node1.edgeList.size());
    

    ROS_INFO("----4.1----%lu", node1.edgeList.size());


    std::vector<Node*> nodeList;
    nodeList.reserve(2);

    ROS_INFO("----4.5----");
    nodeList.push_back(&node0);
    ROS_INFO("----4.6----");
    nodeList.push_back(&node1);
    nodeList.push_back(&node2);
    nodeList.push_back(&node3);
    nodeList.push_back(&node4);
    nodeList.push_back(&node5);
    //nodeList.push_back(node2); 

    ROS_INFO("----5----");

    std::vector<Node*> spath;
    astar(nodeList, spath, 0, 5);


    return 0;
}
