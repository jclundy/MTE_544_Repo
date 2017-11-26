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
	xindex = xValue;
	yindex = yValue;
    xpos = convertToPos(xValue);
    ypos = convertToPos(yValue);

}

Node::Node(const Node &obj)
{
	index = obj.index;
	xindex = obj.xindex;
	yindex = obj.yindex;
	xpos = obj.xpos;
	ypos = obj.ypos;
	edgeList = obj.edgeList;
}

int Node::getIndexOfEdgeWithNode(int otherNodeIndex)
{
	for(int i = 0; i < edgeList.size(); i++)
	{
		if(edgeList[i].endNodeIndex == otherNodeIndex)
		{
			return i;
		}
	}
	return -1;
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

double Node::convertToPos(int a)
{
    return a/10.0; //TBD
}

