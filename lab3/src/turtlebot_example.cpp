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
RViz_Draw drawer_refreshable;
#define GRID_SIZE 100
#define NUM_SAMPLES 250
#define TAGID 0
#define PI 3.14159265
#define SIMULATION
#define DEBUG

float ips_x = 0;
float ips_y = 0;
float ips_yaw;
float forward_v;
float omega;

double occ_grid[GRID_SIZE][GRID_SIZE];
Graph graph;
bool graph_generated = false;

std::vector<Node> checkpoints;

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
  graph.generate_connections(0, 30);
  //graph.print_graph_to_console();
  //std::string np1 = "graph1";
  //std::string np1 = "graph2";
  //graph.draw_in_rviz(publisher,10, 0, 1, 0, 1, "node_samples");
  //ROS_INFO("before pruning edges \n");
  graph.print_graph_to_console();
  graph.prune_invalid_connections(msg, 0.3, 0);
  //ROS_INFO("after pruning edges \n");
  //graph.print_graph_to_console();
  graph.draw_in_rviz(&drawer);
  graph_generated = true;
}

void print_occupancy_grid(double og[][GRID_SIZE], int max_x, int max_y)
{
    for(int i = 0; i < max_x; i++)
    {
        for(int j = 0; j < max_y; j++)
        {
            double prob = og[i][j];
            int val = int(prob / 100);
            int abs_val = abs(val);
            std::cout << val;
        }
        std::cout << ';' << '\n';
    }
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

    drawer.update_map_details(msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y);

    // Reformat input map
    for(int i = 0; i < GRID_SIZE*GRID_SIZE; i++) {
        occ_grid[GRID_SIZE-1 - i/GRID_SIZE][i%GRID_SIZE] = msg.data[i];
    }
    // Place start and end nodes
    drawer.claim(visualization_msgs::Marker::POINTS);
    drawer.update_color(0,1,0,1);
    drawer.update_scale(0.1, 0.1);
    for(int k = 0; k < checkpoints.size(); k++) {
        if(graph.add_new_node(checkpoints[k].xindex, checkpoints[k].yindex)) {
            drawer.add_point_scale(checkpoints[k].xindex, checkpoints[k].yindex);
        }
    }
    drawer.pub();
    drawer.release();
    // Random node placement
    drawer.claim(visualization_msgs::Marker::POINTS);
    drawer.update_color(1,0,0,1);
    drawer.update_scale(0.05, 0.05);
    srand(time(NULL));
    for(int j = 0; j < NUM_SAMPLES - 2; j) {
        int x = rand()%GRID_SIZE;
        int y = rand()%GRID_SIZE;
        if(occ_grid[x][y] == 0 && graph.add_new_node(x, y)) {
            drawer.add_point_scale(x, y);
            j++;
        }
    }

    drawer.pub();
    drawer.release();

    // Publish sampling nodes to RVIZ
    generate_graph(msg, marker_pub);
}

float set_speed(float target_x, float target_y, float prev_theta_error, ros::Publisher velocity_publisher)
{
    float top_speed = 1;
    float kp_theta = 1;
    float kd_theta = 0.5;
    float kp_v = 1;
    //Velocity control variable
    float theta_ref = atan2(target_y - ips_y, target_x - ips_x);
    float theta_error = theta_ref - ips_yaw;
    float dist_error = sqrt((target_y - ips_y)*(target_y - ips_y) +
                            (target_x - ips_x)*(target_x - ips_x));

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
    float forward_v = std::pow(cos_error, 13) * kp_v * dist_error;
    if (forward_v > top_speed)
        forward_v = top_speed;
    if (forward_v < -top_speed)
        forward_v = -top_speed;

    float omega = kp_theta * theta_error + kd_theta * (theta_error - prev_theta_error);

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
    double x_d = a->xpos - b->xpos;
    double y_d = a->ypos - b->ypos;
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
    //ROS_INFO("%s",cstr);
}

void astar(std::vector<Node*>& nodes, std::vector<Node*>& spath, int start_index, int end_index){
    ROS_INFO("Beginning A*");
    ROS_INFO("start node index: %d", nodes[start_index]->index);
    ROS_INFO("End node index: %d", nodes[end_index]->index);
    ROS_INFO("start node x: %f", nodes[end_index]->xindex);
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
    //ROS_INFO("Starting While loop");
    while(!done){
        display_openset_closedset(open_set, closed_set);

        //code for if open set is empty, return an empty closed set
        if(open_set.size() == 0){
            return;
        }

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
            //ROS_INFO("no edges");
        }
        else
        {

            //ROS_INFO("Starting neighbour loop");
            //process each neighbour
            for(int j = 0; j < best_node->edgeList.size(); j++){
                Node* neighbour = nodes[best_node->edgeList[j].endNodeIndex];

                //ROS_INFO("checking if node is in closed set");
                //check if node is in closed set
                bool found = 0;
                for(int k = 0; k < closed_set.size() && !found; k++){
                    if(closed_set[k]->index == neighbour->index){
                        found = 1;
                    }
                }
                if (found)
                {
                    //ROS_INFO("node found in closed set, continuing");
                    continue;
                }else{
                    //ROS_INFO("node not found in closed set");
                }

                //distance to go
                dtogo = distance(nodes[end_index], neighbour);

                //current distance
                dcur = best_node->current_cost + best_node->edgeList[j].cost;
                //ROS_INFO("check if node is in open set and the current distance is lower than previous one");
                //check if node is in open set and the current distance is lower than previous one
                for(int m = 0; m < open_set.size() && !found; m++){

                    if(open_set[m]->index == neighbour->index){
                        found = 1;
                        if(dcur < open_set[m]->current_cost){
                            open_set[m]->back_pointer_index = best_node->index;
                            open_set[m]->lower_bound_cost = dtogo + dcur;
                            open_set[m]->current_cost = dcur;
                        }
                    }
                }
                if(!found){
                    //ROS_INFO("was not found, add to open set");
                    //add endNodeIndex to openSet
                    for(int i = 0; i < nodes.size(); i++){
                        if (neighbour->index == nodes[i]->index){
                            neighbour->back_pointer_index = best_node->index;
                            neighbour->lower_bound_cost = dtogo + dcur;
                            neighbour->current_cost = dcur;
                            open_set.push_back(neighbour);
                        }
                    }
                }
            }

            //ROS_INFO("done neighbour loop. Mindex:%d", mindex);
        }
        //take best_node out of openset
        open_set[mindex] = open_set.back();
        open_set.pop_back();
    }
    //ROS_INFO("Done iterating through graph, finding waypoints");

    Node* temp = best_node;
    while (temp->back_pointer_index != -1)
    {
      spath.push_back(temp);
      temp = nodes[temp->back_pointer_index];
    }

    //ROS_INFO("Finished A* Path:");
    for (int i = 0; i < spath.size(); i++)
    {
        //ROS_INFO("index %d", spath[i]->index);
    }
}



int main(int argc, char **argv)
{
    // Set start and end points
    Node startNode = Node(1, 40, 0);
    Node midNode = Node(2, 80, -40);
    Node endNode = Node(2, 80, 0);

    checkpoints.push_back(startNode);
    checkpoints.push_back(midNode);
    checkpoints.push_back(endNode);

	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;
    ros::NodeHandle n_refreshable;

    Node graphNode(0,0,0);
    ROS_INFO("defined a new node index : %d,%d x: %f y: %f", graphNode.xindex, graphNode.yindex, graphNode.xpos, graphNode.ypos);
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

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    uint wpt_ind = 0;

    bool graph_drawn = false;
    while (ros::ok() && !graph_generated)
    {
        loop_rate.sleep();
        ros::spinOnce();
        ROS_INFO("waiting for graph...");
    }

    std::vector<Node*> nodeList;
    nodeList.reserve(graph.nodeList.size());
    for(int i = 0; i<graph.nodeList.size(); i++)
    {
        nodeList.push_back(&graph.nodeList[i]);
    }

    std::vector<Node*> waypoints;
    //astar(nodeList, waypoints, 0, 25);

 //   Node node0(0,0,0);
  //  Node node1(1, 10, -10);
  //  Node node2(2, 10, 10);
  //  Node node3(3, -10, 10);
  //  Node node4(4, -10, -10);

    astar(nodeList, waypoints, 0, 25);
/*
    waypoints.push_back(&node0);
    waypoints.push_back(&node1);
    waypoints.push_back(&node2);
    waypoints.push_back(&node3);
    waypoints.push_back(&node4);
*/

    uint num_waypoints = waypoints.size();
    if (num_waypoints == 0)
    {
        ROS_WARN("no path found, exiting");
        return 0;
    }

    //TODO: redo with proper scaling once coded
    for(int i = 0; i < num_waypoints; i++) {
        waypoints[i]->xindex *= 0.1;
        waypoints[i]->yindex *= 0.1;
    }


    //draw the path/waypoints
    drawer.claim(visualization_msgs::Marker::LINE_STRIP);
    drawer.update_scale(0.2, 0.2);
    drawer.update_color(1,0,0,1);
    for(int i = 0; i < num_waypoints; i++) {
       drawer.add_point(waypoints[i]->xindex, waypoints[i]->yindex);
    }
    drawer.pub();
    drawer.release();


    float theta_error = 0;
    while (ros::ok())
    {
        drawer_refreshable = RViz_Draw(n_refreshable,"refreshable_visualize_points",true);

        loop_rate.sleep(); //Maintain the loop rate
        ros::spinOnce();   //Check for new messages


        //TODO: END execution here, this is when we are finished traversing the maze
        if (wpt_ind >= num_waypoints)
        {
            wpt_ind = 0;
        }

        float error_mag = get_error_magnitude(waypoints[wpt_ind]->xindex, waypoints[wpt_ind]->yindex);
        theta_error = set_speed(waypoints[wpt_ind]->xindex, waypoints[wpt_ind]->yindex, theta_error, velocity_publisher);
        //ROS_INFO("theta_error = %f,   x,y = %f \t %f    dist=%f", theta_error, ips_x, ips_y, error_mag);



        //draw robot position
        drawer_refreshable.claim(visualization_msgs::Marker::POINTS);
        drawer_refreshable.update_color(1, 0, 1, 1);
        drawer_refreshable.update_scale(0.5, 0.5);
        drawer_refreshable.add_point(ips_x, ips_y);
        drawer_refreshable.pub();
        drawer_refreshable.release();

        //draw next waypoint
        drawer_refreshable.claim(visualization_msgs::Marker::POINTS);
        drawer_refreshable.update_color(0.1, 1, 0.1, 1);
        drawer_refreshable.add_point(waypoints[wpt_ind]->xindex, waypoints[wpt_ind]->yindex);
        drawer_refreshable.pub();
        drawer_refreshable.release();

        //draw robot facing direction marker
        drawer_refreshable.claim(visualization_msgs::Marker::POINTS);
        drawer_refreshable.update_color(1, 0, 0, 1);
        drawer_refreshable.update_scale(0.2, 0.2);
        drawer_refreshable.add_point(ips_x + 0.5*cos(ips_yaw),ips_y + 0.5*sin(ips_yaw));
        drawer_refreshable.pub();
        drawer_refreshable.release();

        if (error_mag < 0.08)
        {
            wpt_ind++;
        }
    }
    return 0;
}
