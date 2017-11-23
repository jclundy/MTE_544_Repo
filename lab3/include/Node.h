#ifndef NODE_H
#define NODE_H

#include "Edge.h"
#include <vector>

class Node
{
public:
	int index;
	double x;
	double y;
	
	int back_pointer_index;
	double lower_bound_cost; 
	double current_cost;

	std::vector<Edge> edgeList;
	void addEdge(Edge edge);
	void removeEdge(int edgeIndex);
	bool isConnectedToNodeAtIndex(int nodeIndex);

	Node(int indexValue, double xValue, double yValue);
	
};

#endif
