#ifndef NODE_H
#define NODE_H

#include "Edge.h"
#include <vector>

class Node
{
public:
    int index;
    int xindex;
    int yindex;
	double xpos;
	double ypos;

	int back_pointer_index;
	double lower_bound_cost;
	double current_cost;

	std::vector<Edge> edgeList;
	void addEdge(Edge edge);
	bool removeEdge(int edgeIndex);
	bool isConnectedToNodeAtIndex(int nodeIndex);
    double convertToPos(int a);
    int getIndexOfEdgeWithNode(int otherNodeIndex);

    Node();
	Node(int indexValue, int xValue, int yValue);
	Node(const Node &obj);

};

#endif
