#ifndef GRAPH_H
#define GRAPH_H

#include "Graph.h"
#include "Edge.h"
#include "Node.h"
#include "rviz_draw.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <vector>
#include <cstdlib>

class Graph
{
public:
	int startIndex;
	int endIndex;
	std::vector<Node> nodeList;

	Graph();

	Graph(int start, int end, std::vector<Node> &listOfNodes);

	void generate_connections(int connectionsPerNode, double maxDistance);

	void prune_invalid_connections(nav_msgs::OccupancyGrid map, double robotSize, double isEmptyValue);

    bool add_new_node(int x, int y);

    void draw_in_rviz(RViz_Draw *drawer);

    void print_graph_to_console();

    double calculate_distance(int i, int j);

private:
	double calculate_distance(Node &start, Node &end);

	int convertPositionToGridIndex(double position, double mapLowerLimit, double resolution);

	bool isConnectionValid(Node &startNode, Node &endNode, nav_msgs::OccupancyGrid& map, double robotSize, double isEmptyValue);

	void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y);
};

#endif
