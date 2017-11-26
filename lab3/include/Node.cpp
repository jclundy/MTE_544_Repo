#include "Node.h"

Node::Node()
{
    index = -1;
    xindex = 0;
    yindex = 0;
    xpos = 0;
    ypos = 0;
}

Node::Node(int indexValue, int xValue, int yValue)
{
    index = indexValue;
    setPos(xValue, yValue);
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

void Node::setIndex(int x, int y)
{
    xindex = x;
    yindex = y;
    xpos = x/10.0;
    ypos = y/10.0 - 5;
}

void Node::setPos(double x, double y)
{
    xpos = x;
    ypos = y;
    xindex = x*10;
    yindex = (y+5)*10;
}
