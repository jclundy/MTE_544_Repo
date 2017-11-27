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
	void removeEdge(int edgeIndex);
	bool isConnectedToNodeAtIndex(int nodeIndex);
    void setIndex(int x, int y);
    void setPos(double x, double y);

    Node();
	Node(int indexValue, double xValue, double yValue, bool isIndex);

};

#endif
