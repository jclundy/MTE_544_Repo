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
	std::vector<Edge> edgeList;
	void addEdge(Edge edge);
	void removeEdge(int edgeIndex);
	bool isConnectedToNodeAtIndex(int nodeIndex);

	Node(int indexValue, double xValue, double yValue);
	
};

#endif
