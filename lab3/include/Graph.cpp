#include "Graph.h"

short sgn(int x) { return x >= 0 ? 1 : -1; }

Graph::Graph()
{
	startIndex = 0;
	endIndex = 0;
}

Graph::Graph(int start, int end, std::vector<Node> &listOfNodes)
{
	startIndex = start;
	endIndex = end;
	nodeList = listOfNodes;

}

void Graph::generate_connections(int connectionsPerNode, double maxDistance)
{
	// Idea 1: just add all nodes as a connection that are less than X distance away
	// iterate through list of nodes
	for(int i = 0; i < nodeList.size(); i++)
	{
		for(int j = i + 1; j < nodeList.size(); j++)
		{
			// check that an edge doesn't already exist between the nodes
			if(nodeList[i].isConnectedToNodeAtIndex(j)) continue;

			// check that nodes are close enough
			double distance = calculate_distance(nodeList[i], nodeList[j]);
			if(distance < maxDistance)
			{
				// update both nodes with an edge to each other
				Edge edgeToStart = Edge(distance,i);
				Edge edgeToEnd = Edge(distance, j);
				nodeList[i].addEdge(edgeToEnd);
				nodeList[j].addEdge(edgeToStart);
			}
		}
	}
}

double Graph::calculate_distance(Node start, Node end)
{
	double dx = end.x - start.x;
	double dy = end.y - start.y;
	return std::sqrt(dx*dx + dy*dy);
}

void Graph::prune_invalid_connections(nav_msgs::OccupancyGrid map , double robotSize, double isEmptyValue)
{
	// Idea: iterate through list of nodes
	// for each node, iterate through its list of edges
	// for a valid connection, mark as checked (on current node, and as well as on endpoint node)
	// remove invalid connections
	for(int i = 0; i < nodeList.size(); i++)
	{
		for(int j = 0; j < nodeList[i].edgeList.size(); j++)
		{
			// if edge has already been checked skip to next edge
			Node startNode = nodeList[i];
			Edge startEdge = startNode.edgeList[j];
			int endNodeIndex = startEdge.endNodeIndex;

			if(startEdge.validated) continue;
			Node endNode = nodeList[endNodeIndex];

			// find corresponding edge in other node
			int indexOfEdgeInEndNode = getIndexOfEdgeWithNode(endNode,i);
			// skip if edge to given node is not found
			if(indexOfEdgeInEndNode < 0)
			{
				//ROS_INFO("edge %i not found in node %i",indexOfEdgeInEndNode, endNodeIndex);
				nodeList[i].removeEdge(j);
				continue;
			}
			// check if the edge results in a collision

			bool isCollisionFree = isConnectionValid(startNode, endNode, map, robotSize, isEmptyValue);
			if(isCollisionFree)
			{
				// mark edge as validated
				nodeList[i].edgeList[j].validated = true;
				// mark corresponding edge in other node as validated
				nodeList[endNodeIndex].edgeList[indexOfEdgeInEndNode].validated = true;
			} else {
				// remove edges from both start and end nodes
				//ROS_INFO("Removing Invalid Connection %i, %i", i, endNodeIndex);
				nodeList[i].removeEdge(j);
				nodeList[endNodeIndex].removeEdge(indexOfEdgeInEndNode);
			}
		}
	}
}

int Graph::getIndexOfEdgeWithNode(Node node, int otherNodeIndex)
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

int Graph::convertPositionToGridIndex(double position, double mapLowerLimit, double resolution)
{
	return int((position - mapLowerLimit)/resolution);
}

bool Graph::isConnectionValid(Node startNode, Node endNode, nav_msgs::OccupancyGrid& map, double robotSize, double isEmptyValue)
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

	double xMapStart = map.info.origin.position.x;
	double yMapStart = map.info.origin.position.y;
	double mapResolution = map.info.resolution;

	int x0 = int(startNode.x);
	int y0 = int(startNode.y);

	int x1 = int(endNode.x);
	int y1 = int(endNode.y);

	std::vector<int> xTileIndices;
	std::vector<int> yTileIndices;
	bresenham(x0, y0, x1, y1, xTileIndices, yTileIndices);

	int numberOfTiles = xTileIndices.size();

	int padding = 0;
	bool checkVertical = 0;
	bool checkHorizontal = 0;
	/*if(robotSize > mapResolution)
	{
		// need to check additional tiles
		int ratio = robotSize/mapResolution;
		// round ratio up to odd value
		if(ratio % 2 == 0) ratio += 1;
		// this is how many tiles above/below or right/left center tile that will be checked as well
		padding = (ratio - 1)/2;

		int dx = std::abs(x1 - x0);
		int dy = std::abs(y1 - y0);
		if(dx >= dy)
		{
			// edge is roughly horizontal, perform check on tiles above and below center tile
			checkVertical = 1;
		} else {
			// edge is roughly vertical, perform check on tiles to the left and right of center tile
			checkHorizontal = 1;
		}
	}*/
	for(int i = 0; i < numberOfTiles; i++)
	{
		int xIndex = xTileIndices[i];
		int yIndex = yTileIndices[i];
		int mapDataIndex = xIndex + yIndex * map.info.width;
		if(map.data[mapDataIndex] > isEmptyValue)
		{
			//ROS_INFO("Invalid Connection %i, %i, with value %d", startNode.index, endNode.index, map.data[mapDataIndex]);
			return false;
		}
		// iterate through tiles adjacent to
		/*for(int j = -padding; j<=padding; j++)
		{
			// calculate index of tiles avbove/below
			int modifiedXIndex = xIndex + j * checkHorizontal;
			int modifiedYIndex = yIndex + j * checkVertical;
			int mapDataIndex = modifiedXIndex + modifiedYIndex * 100;  //TODO THIS SHOULD BE PASSED IN
			// skip check if index is out of bounds
			int size = map.data.size();
			if(mapDataIndex > size - 1)
			{
				ROS_INFO("mapDataIndex %i out of bounds %i",mapDataIndex, size);
				continue;
			}

			// if the value at mapDataIndex is occupied, there is a collision
			if(map.data[mapDataIndex] >= isEmptyValue)
			{
				return false;
			}
		}*/
	}
	// no obstacles were detected along the given edge
	return true;
}


void Graph::bresenham(int x0, int y0, int x1, int y1, std::vector<int>& x, std::vector<int>& y) {

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


bool Graph::add_new_node(int x, int y) {
	int i;
    for(i = 0; i<nodeList.size(); i++) {
        if(nodeList[i].x == x && nodeList[i].y == y)
            return false;
    }

    Node new_node = Node(i, x, y);
    nodeList.push_back(new_node);
    return true;
}

void Graph::draw_in_rviz(RViz_Draw *drawer)
{
    drawer->claim(visualization_msgs::Marker::LINE_STRIP);
    drawer->update_color(0, 0, 1, 1);
    for (int i = 0; i < nodeList.size(); i++)
    {
      for(int j = 0; j < nodeList[i].edgeList.size(); j++)
      {
      	int endNodeIndex = nodeList[i].edgeList[j].endNodeIndex;
		double x0 = nodeList[i].x;
      	double y0 = nodeList[i].y;
      	double x1 = nodeList[endNodeIndex].x;
      	double y1 = nodeList[endNodeIndex].y;
      	/*geometry_msgs::Point p0, p1;
		p0.x = x0*0.1;
		p0.y = y0*0.1;
		p0.z = 0; //not used
		p1.x = x1 * 0.1;
		p1.y = y1 * 0.1;
		p1.z = 0;
      	lines.points.push_back(p0);
		lines.points.push_back(p1);
            */
        drawer->add_point_scale(x0, y0);
        drawer->add_point_scale(x1, y1);
      }
    }
    drawer->pub();
    drawer->release();


}

void Graph::print_graph_to_console()
{
	for(int i = 0; i < nodeList.size(); i++)
	{
		//ROS_INFO("Node %i ", nodeList[i].index);
		//ROS_INFO("X %f, Y %f \n", nodeList[i].x, nodeList[i].y);
		int size = nodeList[i].edgeList.size();
		//ROS_INFO("Number of edges: %i \n", size);
		/*for(int j = 0; j < size; j++)
		{
			ROS_INFO("Edge to Node %i, cost %f \n", nodeList[i].edgeList[j].endNodeIndex, nodeList[i].edgeList[j].cost);
		}*/
	}
}
