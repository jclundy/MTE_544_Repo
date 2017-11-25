#ifndef EDGE_H
#define EDGE_H

//#include "Node.h"
class Edge
{
public:
	int endNodeIndex;
	double cost;
	bool validated;
	
	Edge(double costValue, int endNodeIndex);
	void validate();
	Edge (const Edge &obj);
};

#endif
