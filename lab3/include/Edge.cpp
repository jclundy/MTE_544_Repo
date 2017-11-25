#include "Edge.h"

Edge::Edge(double costValue, int endNodeIndexValue)
{
	validated = false;
	cost = costValue;
	endNodeIndex = endNodeIndexValue;
}

void Edge::validate()
{
	//TODO implement this validation function
	// requires grid map 
}
Edge::Edge (const Edge &obj)
{
	validated = obj.validated;
	cost = obj.cost;
	endNodeIndex = obj.endNodeIndex;
}