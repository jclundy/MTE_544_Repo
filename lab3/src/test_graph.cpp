//  ///////////////////////////////////////////////////////////
//
// test_graph.cpp
//
//
// Author: Optimus Prime
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
#define NUM_SAMPLES 15
#define TAGID 0
#define PI 3.14159265
#define SIMULATION
#define DEBUG

float ips_x = 0;
float ips_y = 0;
float ips_yaw;
float forward_v;
float omega;
bool map_drawn = false;

double occ_grid[GRID_SIZE][GRID_SIZE];
// int TEST_GRAPH_NODE_COUNT = 11;
// int TEST_GRAPH_X[11] = {0, 80, 80, 30, 7, 94, 85, 15, 1, 52, 14};
// int TEST_GRAPH_Y[11] = {50, 10, 50, 1, 67, 64, 12, 97, 37, 43, 11};

#define TEST_GRAPH_NODE_COUNT 13
int TEST_GRAPH_X[TEST_GRAPH_NODE_COUNT] = {97,45,78,24,1,49,19,73,61,62,11,89,44};
int TEST_GRAPH_Y[TEST_GRAPH_NODE_COUNT] = { 68,39,58,84,78,96,85,98,3,75,16,9,25};
bool graph_generated = false;
Graph graph;
nav_msgs::OccupancyGrid og_msg;

void print_occupancy_grid(double og[][GRID_SIZE], int max_x, int max_y)
{
    for(int j = 0; j < max_y; j++)
    {
        for(int i = 0; i < max_x; i++)
        {
            double prob = og[i][j];
            int val = int(prob / 100);
            int abs_val = abs(val);
            std::cout << val;
        }
        std::cout << ';' << '\n';
    }
}

void print_map_data(const nav_msgs::OccupancyGrid& msg)
{
	std::cout << "================== map array ======================== \n";
	for(int i = 0; i < GRID_SIZE*GRID_SIZE; i++)
	{
		int val = msg.data[i]/100;
		std::cout << val << ",";
	}
	std::cout << "================== end of map array ======================== \n";
}

void generate_graph(const nav_msgs::OccupancyGrid& msg, ros::Publisher publisher)
{
  ROS_INFO("before generating edges \n");
  //graph.print_graph_to_console();
  graph.generate_connections(0, 30);
  ROS_INFO("after generating edges \n");
  //graph.print_graph_to_console();
  //std::string np1 = "graph1";
  //std::string np1 = "graph2";
  graph.print_graph_to_console();
  //drawer.update_color(0, 1, 0, 1);
  //graph.draw_in_rviz(&drawer);
  graph.prune_invalid_connections(msg, 0.3, 0);
  ROS_INFO("after pruning edges \n");
  graph.print_graph_to_console();
  drawer.update_color(0, 0, 1, 1);
  graph.draw_in_rviz(&drawer);
  graph_generated = true;
}

void map_callback(const nav_msgs::OccupancyGrid& msg)
{
    // Assuming 100x100 map input, will complain if that doesn't match
    // Grid is occ[x][y] increasing with axis
    og_msg = msg;
    if(msg.info.width != GRID_SIZE || msg.info.height != GRID_SIZE) {
        ROS_INFO("actual width %d, height %d, vs GRID_SIZE %i", msg.info.width, msg.info.height, GRID_SIZE);
        ROS_INFO("Inconsistent map sizes, dumping...");
        return;
    }

    drawer.update_map_details(msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y-5);

    // Reformat input map
    for(int i = 0; i < GRID_SIZE*GRID_SIZE; i++) {
        occ_grid[i/GRID_SIZE][i%GRID_SIZE] = msg.data[i];
    }
    // Place start and end nodes
    drawer.claim(visualization_msgs::Marker::POINTS);
    drawer.update_color(0,1,0,1);
    drawer.update_scale(0.1, 0.1);
    // for(int k = 0; k < checkpoints.size(); k++) {
    //     if(graph.add_new_node(checkpoints[k].xindex, checkpoints[k].yindex)) {
    //         drawer.add_point_scale(checkpoints[k].xindex, checkpoints[k].yindex);
    //     }
    // }
    drawer.pub();
    drawer.release();
    // Random node placement
    drawer.claim(visualization_msgs::Marker::POINTS);
    drawer.update_color(1,0,0,1);
    drawer.update_scale(0.05, 0.05);
    for(int j = 0; j < TEST_GRAPH_NODE_COUNT; j++) {
        int x = TEST_GRAPH_X[j];
        int y = TEST_GRAPH_Y[j];
        Node n = Node(graph.nodeList.size(), x, y, 1);
        if(occ_grid[x][y] == 0 && graph.add_new_node(n)) {
        	ROS_INFO("adding node %i (%i, %i)", j, x, y);
            drawer.add_node(n);
        }
    }

    // srand(time(NULL));
    // for(int j = 0; j < NUM_SAMPLES - 2; j) {
    //     int x = rand()%GRID_SIZE;
    //     int y = rand()%GRID_SIZE;
    //     if(occ_grid[x][y] == 0 && graph.add_new_node(x, y)) {
    //         drawer.add_point_scale(x, y);
    //         j++;
    //     }
    // }

    drawer.pub();
    drawer.release();

    //map_print(occ_grid);

    // Publish sampling nodes to RVIZ
    generate_graph(msg, marker_pub);
    map_drawn = true;
}

void initialize_test_graph(Graph &test_graph)
{
	for(int i = 0; i < TEST_GRAPH_NODE_COUNT; i ++)
	{
		double x_correct = TEST_GRAPH_X[i];
		double y_correct = TEST_GRAPH_Y[i];
		Node n = Node(test_graph.nodeList.size(), x_correct, y_correct ,1);
		bool added_new_node = test_graph.add_new_node(n);
		if(!added_new_node)
		{
			ROS_INFO("failed to add new node: %i at %f, %f", i, x_correct, y_correct);
		}
	}
}

bool test_graph_initialization(Graph &test_graph)
{
	// make sure values intialized correctly
	bool test_passed = true;
	for(int i = 0; i < TEST_GRAPH_NODE_COUNT; i ++)
	{
		int index = test_graph.nodeList[i].index;
		if(index != i)
		{
			ROS_INFO("Index of node %i not equal to its index field %i", i, index);
			test_passed = false;
		}
		int x_correct = TEST_GRAPH_X[i];
		int y_correct = TEST_GRAPH_Y[i];

		int x = test_graph.nodeList[i].xindex;
		int y = test_graph.nodeList[i].yindex;
		if(x != x_correct)
		{
			ROS_INFO("X value %i of node %i not equal to intialized X %i", i, x, x_correct);
			test_passed = false;
		}
		if(y != y_correct)
		{
			ROS_INFO("Y value %i of node %i not equal to intialized Y %i", i, y, y_correct);
			test_passed = false;
		}		
	}
	ROS_INFO("test_graph_initialization passed : %i", test_passed);
	return test_passed;
}

bool test_graph_edge_generation(Graph &test_graph, double max_distance)
{
	bool test_passed = true;	
	// test that all edges of node are within specified distance
	for(int i = 0; i < TEST_GRAPH_NODE_COUNT; i++)
	{
		for(int j = 0; j < test_graph.nodeList[i].edgeList.size(); j++)
		{

			double cost = test_graph.nodeList[i].edgeList[j].cost;
			int endNodeIndex = test_graph.nodeList[i].edgeList[j].endNodeIndex;
			if(test_graph.nodeList[i].edgeList[j].cost > max_distance)
			{
				ROS_INFO("Cost %f of edge %i, %i greater than allowed max distance %f", cost, i, endNodeIndex, max_distance);
				test_passed = false;
			}
		}	
	}
	
	for(int i = 0; i < TEST_GRAPH_NODE_COUNT; i++)
	{
		for (int j = i+1; j < TEST_GRAPH_NODE_COUNT; j++)
		{
			int x0 = test_graph.nodeList[i].xindex;
			int y0 = test_graph.nodeList[i].yindex;
			int x1 = test_graph.nodeList[j].xindex;
			int y1 = test_graph.nodeList[j].yindex;

			double distance = test_graph.calculate_distance(i, j);
			ROS_INFO("Node %i : (%i,%i) and Node %i, (%i,%i)", i, x0, y0, j, x1, y1);

			// test that all nodes within specified distance form edges
			if(distance < max_distance)
			{
				// check edge from i -> j
				if(!test_graph.nodeList[i].isConnectedToNodeAtIndex(j))
				{
					ROS_INFO("Node %i not connected to node %i", i, j);
					test_passed = false;
				}
				if(!test_graph.nodeList[j].isConnectedToNodeAtIndex(i))
				{
					ROS_INFO("Node %i not connected to node %i", j, i);
					test_passed = false;
				}
			}
		}
	}

	for(int i = 0; i < TEST_GRAPH_NODE_COUNT; i++)
	{
		for(int j = 0; j < test_graph.nodeList[i].edgeList.size(); j++)
		{

			double cost = test_graph.nodeList[i].edgeList[j].cost;
			int endNodeIndex = test_graph.nodeList[i].edgeList[j].endNodeIndex;
			double distance = test_graph.calculate_distance(i, endNodeIndex);
			if(test_graph.nodeList[i].edgeList[j].cost != distance)
			{
				ROS_INFO("Cost %f of edge %i, %i does not equal distance %f between nodes", cost, i, endNodeIndex, distance);
				test_passed = false;
			}
		}	
	}
	ROS_INFO("test_graph_edge_generation passed : %i", test_passed);
	return test_passed;
}

bool test_edge_removal()
{
	ROS_INFO("Test edge removal");
	int startIndex = 0;
	int endIndex = 1;

	Node start(0,0,startIndex, 1);
	Edge startEdge = Edge(1.41, endIndex);
	start.addEdge(startEdge);
	Node end(1, 1, endIndex, 1);
	Edge endEdge = Edge(1.41, startIndex);
	end.addEdge(endEdge);

	int endEdgeIndex = end.getIndexOfEdgeWithNode(startIndex);
	int startEdgeIndex = start.getIndexOfEdgeWithNode(endIndex);
	bool test_passed = true;

	if(!start.removeEdge(endEdgeIndex))
	{
		ROS_INFO("Failed to remove edge %i from node", endEdgeIndex);
		test_passed = false;
	}

	if(start.getIndexOfEdgeWithNode(endIndex) != -1)
	{
		ROS_INFO("Failed to remove connection from start node");
		test_passed = false;
	}

	if(!end.removeEdge(startEdgeIndex))
	{
		ROS_INFO("Failed to remove edge %i from node", startEdgeIndex);
		test_passed = false;
	}

	if(end.getIndexOfEdgeWithNode(startIndex) != -1)
	{
		ROS_INFO("Failed to remove connection from end node");
		test_passed = false;
	}
	ROS_INFO("test_edge_removal passed : %i", test_passed);
	return test_passed;

}

int main(int argc, char **argv)
{
	

	Graph test_graph;
	

	bool done = false;
	ros::init(argc,argv,"main_control");
    ros::NodeHandle n;
    drawer = RViz_Draw(n);
	ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
	ROS_INFO("Beginning of graph tests");
	ros::Rate loop_rate(20); 
	while (ros::ok() && !done)
	{
	    loop_rate.sleep();
	    ros::spinOnce();
	    if(map_drawn) done = true;
	}
	print_occupancy_grid(occ_grid, GRID_SIZE, GRID_SIZE);
	print_map_data(og_msg);
	// ROS_INFO("Initializing test graph");
	// initialize_test_graph(test_graph);

	// ROS_INFO("Test 1 - graph properly intialized");
	// test_graph_initialization(test_graph);

	// ROS_INFO("Test 2 - edges properly generated");
	// double max_distance = GRID_SIZE*1.41; // distance from opposite corners 
	// test_graph.generate_connections(TEST_GRAPH_NODE_COUNT, max_distance);
	// test_graph_edge_generation(test_graph, max_distance);

	// ROS_INFO("Test 3 - testing edge removal");
	// test_edge_removal();

    ROS_INFO("End of graph tests");
	return 0;
}