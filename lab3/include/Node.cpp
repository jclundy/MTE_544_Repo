#include "Node.h"
Node::Node(int indexValue, double xValue, double yValue)
{
	index = indexValue;
	x = xValue;
	y = yValue;
	
}

void Node::addEdge(Edge edge)
{
	edgeList.push_back(edge);
}

void Node::removeEdge(int edgeIndex)
{
	// order is not important, 
	// so we overwrite the edge at 
	// edgeIndex with the last edge in the list
	// then pop off the last node in the list
	edgeList[edgeIndex] = edgeList.back();
	edgeList.pop_back();
}

bool Node::isConnectedToNodeAtIndex(int nodeIndex)
{
	for(int i = 0; i < edgeList.size(); i++)
	{
		if(edgeList[i].endNodeIndex == nodeIndex)
		{
			return true;
		}
	}
	return false;
}

