#include "Edge.h"
#include "Node.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <math.h>

void generate_connections(vector<Node> graph, int connectionsPerNode, double maxDistance)
{
	// Idea 1: just add all nodes as a connection that are less than X distance away	
	// iterate through list of nodes
	for(int i = 0; i < graph.size(); i++)
	{
		for(int j = 0; j < graph.size(); j++)
		{
			// skip creating connections to same node
			if (i = j) continue;
			
			// check that an edge doesn't already exist between the nodes
			if(graph[i].isConnectedToNodeAtIndex(j)) continue;

			// check that nodes are close enough
			double distance = calculate_distance(graph[i], graph[j]);
			if(distance < maxDistance)
			{
				// update both nodes with an edge to each other
				Edge edgeToStart = new Edge(distance,i);
				Edge edgeToEnd = new Edge(distance, j);
				graph[i].addEdge(edgeToEnd);
				graph[j].addEdge(edgeToStart);
			}
		}
	}
}

double calculate_distance(Node start, Node end)
{
	dx = end.x - start.x;
	dy = end.y = start.y;
	return std::sqrt(dx*dx + dy*dy);
}

double prune_invalid_connections(vector<Node> graph, nav_msgs::OccupancyGrid map, double robotSize, int isOccupiedThreshold)
{
	// Idea: iterate through list of nodes
	// for each node, iterate through its list of edges
	// for a valid connection, mark as checked (on current node, and as well as on endpoint node)
	// remove invalid connections
	for(int i = 0; i < graph.size(); i++)
	{
		for(int j = 0; j < graph[i].edgeList.size(); j++)
		{
			// if edge has already been checked skip to next edge
			Node startNode = graph[i];
			Edge startEdge = startNode.edgeList[j];

			if(startEdge.isValidated) continue;
			int endIndex = startEdge.endNodeIndex;
			Node endNode = graph[endNodeIndex];
			Edge endEdge;

			// find corresponding edge in other node
			int indexOfEdgeInEndNode = getIndexOfEdgeWithNode(endNode,i);
			// check if the edge results in a collision
			bool isCollisionFree = isConnectionValid(startNode, endNode, map, robotSize, isOccupiedThreshold);
			if(isCollisionFree)
			{
				// mark edge as validated
				graph[i].edgeList[j].isValidated = true;
				// skip if edge to given node is not found
				if(indexOfEdgeInEndNode < 0) continue;
				// mark corresponding edge in other node as validated 
				graph[endIndex].edgeList[indexOfEdgeInEndNode].isValidated = true;
			} else {
				// remove edges from both start and end nodes
				graph[i].removeEdge(j);
				graph[endIndex].removeEdge(indexOfEdgeInEndNode);
			}
		}
	}
}

int getIndexOfEdgeWithNode(Node node, int otherNodeIndex)
{
	for(int i = 0; i < node.edgeList.size(); i++)
	{
		if(node.edgeList[i].endNodeIndex == otherNodeIndex)
		{
			return i;
		}
	}
	return -1;
}

int convertPositionToGridIndex(double position, double mapLowerLimit, double resolution)
{
	return int((position - mapLowerLimit)/resolution);
}

bool isConnectionValid(Node startNode, Node endNode, nav_msgs::OccupancyGrid map, double robotSize, int isOccupiedThreshold)
{
	// Determining if connection valid:
	// 1. map start and end node x-y position to map x-y indices
	// 2. Use line-drawing algorithm, get list of tiles 
	// 3. check each tile in the list if it is occupied
	// 4. if tile in list is occupied connection is not valid
	// 
	// If robot is wider than tile width:
	// 1. determine if edge is more horizontal/vertical
	// 2. for ~horizontal:
	// 		check N tiles above/below tile in list from step 2 in section above
	// for vertical:
	// check N tiles to left/right of tile in list from above
	// for diagonal, check
	double xMapStart = map.info.origin.position.x;
	double yMapStart = map.info.origin.position.y;
	double mapResolution = map.info.resolution;
	
	int x0 = convertPositionToGridIndex(startNode.x,xMapStart,mapResolution);
	int y0 = convertPositionToGridIndex(startNode.y,yMapStart,mapResolution);

	int x1 = convertPositionToGridIndex(endNode.x,xMapStart,mapResolution);
	int y1 = convertPositionToGridIndex(endNode.y,yMapStart,mapResolution);

	std::vector<int> xTileIndices;
	std::vector<int> yTileIndices;
	bresenham(x0, y0, x1, y1, xTileIndices, yTileIndices);

	int numberOfTiles = xTileIndices.size();
	for(int i = 0; i < numberOfTiles; i++)
	{
		int xIndex = xTileIndices[i];
		int yIndex = yTileIndices[i];
		int mapDataIndex = xIndex + GRID_SIZE * yIndex;	
		
		// if the value at index is non-zero, there is a collision
		//TODO VERIFY MAP GIVEN IS BINARY 
		if(map.data[mapDataIndex] >= isOccupiedThreshold)
			return false;
	}
	// no obstacles along path in the grid
	return true;
}

void bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y) {

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;

    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s) y0+=sgn(dy2); else x0+=sgn(dx2);
        if (d < 0) d += inc1;
        else {
            d += inc2;
            if (s) x0+=sgn(dx2); else y0+=sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}
