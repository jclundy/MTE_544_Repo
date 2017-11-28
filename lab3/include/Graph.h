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
    int graphSize;
    double resolution;

	Graph();

	Graph(int start, int end, std::vector<Node> &listOfNodes);

	void generate_connections(int connectionsPerNode, double maxDistance);

	void prune_invalid_connections(double occ_grid[][100], double robotSize, double isEmptyValue);

    bool add_new_node(Node n);

    void draw_in_rviz(RViz_Draw *drawer);

    void print_graph_to_console();

    double calculate_distance(int i, int j);

    bool areAllEdgesValidated();

private:
	double calculate_distance(Node &start, Node &end);

	bool isConnectionValid(int startIndex, int endIndex, double occ_grid[][100], double robotSize, int isEmptyValue);

	void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y);
};

#endif
