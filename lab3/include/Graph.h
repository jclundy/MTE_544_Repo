#ifndef GRAPH_H
#define GRAPH_H

#include "Graph.h"
#include "Edge.h"
#include "Node.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <math.h>
#include <vector>

class Graph
{
public:
	int startIndex;
	int endIndex;
	std::vector<Node> nodeList;

	Graph();

	Graph(int start, int end, std::vector<Node> &listOfNodes);

	void generate_connections(int connectionsPerNode, double maxDistance);

	void prune_invalid_connections(nav_msgs::OccupancyGrid map, double robotSize, double isOccupiedThreshold);

    bool add_new_node(int x, int y);

private:
	double calculate_distance(Node start, Node end);

	int getIndexOfEdgeWithNode(Node node, int otherNodeIndex);

	int convertPositionToGridIndex(double position, double mapLowerLimit, double resolution);

	bool isConnectionValid(Node startNode, Node endNode, nav_msgs::OccupancyGrid map, double robotSize, double isOccupiedThreshold);

	void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y);
};

#endif
