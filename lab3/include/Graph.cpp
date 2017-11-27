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

double Graph::calculate_distance(Node &start, Node &end)
{
	double dx = end.xindex - start.xindex;
	double dy = end.yindex - start.yindex;
	return std::sqrt(dx*dx + dy*dy);
}
double Graph::calculate_distance(int i, int j)
{
	double dx = nodeList[i].xindex - nodeList[j].xindex;
	double dy = nodeList[i].yindex - nodeList[j].yindex;
	return std::sqrt(dx*dx + dy*dy);
}

void Graph::prune_invalid_connections(nav_msgs::OccupancyGrid map, double robotSize, double isEmptyValue)
{
	// Idea: iterate through list of nodes
	// for each node, iterate through its list of edges
	// for a valid connection, mark as checked (on current node, and as well as on endpoint node)
	// remove invalid connections
//#ifdef DEBUG
	////ROS_INFO("Pruning Edges");
	////ROS_INFO("Map width: %i, Map heigth: %i", map.info.width, map.info.height);
//#endif
	for(int startIndex = 0; startIndex < nodeList.size(); startIndex++)
	{
		//ROS_INFO("Node %i", startIndex);
		for(int startEdgeIndex = 0; startEdgeIndex < nodeList[startIndex].edgeList.size(); startEdgeIndex++)
		{
			// if edge has already been checked skip to next edge
			//ROS_INFO("Edge %i", startEdgeIndex);
			int endIndex = nodeList[startIndex].edgeList[startEdgeIndex].endNodeIndex;

			//ROS_INFO("End Node %i", endIndex);

			//ROS_INFO("Edge %i", startEdgeIndex);
			if(nodeList[startIndex].edgeList[startEdgeIndex].validated) continue;

			// find corresponding edge in other node
			int endEdgeIndex = nodeList[endIndex].getIndexOfEdgeWithNode(startIndex);
			//ROS_INFO("Edge in EndNode %i", endEdgeIndex);
			// skip if edge to given node is not found
			if(endEdgeIndex < 0)
			{
				//ROS_INFO("edge %i not found in node %i",endEdgeIndex, endIndex);
				nodeList[startIndex].removeEdge(startEdgeIndex);
				continue;
			}
			// check if the edge results in a collision
			//ROS_INFO("Checking for collision");
			bool isCollisionFree = isConnectionValid(startIndex, endIndex, map, robotSize, isEmptyValue);
			if(isCollisionFree)
			{
				// mark edge as validated
				//ROS_INFO("Collision Free");
				nodeList[startIndex].edgeList[startEdgeIndex].validated = true;
				//ROS_INFO("Marked edge on start node as validated");
				// mark corresponding edge in other node as validated
				nodeList[endIndex].edgeList[endEdgeIndex].validated = true;
				//ROS_INFO("Marked edge on end node as validated");
			} else {
				// remove edges from both start and end nodes
				////ROS_INFO("Removing Invalid Connection %i, %i", i, endIndex);
				//ROS_INFO("Has Collision");
				nodeList[startIndex].removeEdge(startEdgeIndex);
				//ROS_INFO("Removed edge from start node");
				nodeList[endIndex].removeEdge(endEdgeIndex);
				//ROS_INFO("Removed edge from end node");
			}
			std::cout << "----------------------------------------\n"; 
		}
		std::cout << "===========================================\n"; 
	}
}



bool Graph::isConnectionValid(int startIndex, int endIndex, nav_msgs::OccupancyGrid& map, double robotSize, int isEmptyValue)
{
	//Node &startNode, Node &endNode
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

	int x0 = nodeList[startIndex].xindex;
	int y0 = nodeList[startIndex].yindex;
	//ROS_INFO("Start Node %i (%i, %i)", startIndex, x0, y0);

	int x1 = nodeList[endIndex].xindex;
	int y1 = nodeList[endIndex].yindex;
	//ROS_INFO("End Node %i (%i, %i)", endIndex, x1, y1);

	ROS_INFO("Start Node %i (%i, %i); End Node %i (%i, %i)", startIndex, x0, y0, endIndex, x1, y1);
	std::vector<int> xTileIndices;
	std::vector<int> yTileIndices;
	bresenham(x0, y0, x1, y1, xTileIndices, yTileIndices);
	//ROS_INFO("Running bresenham line algorithm");

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
		int mapDataIndex = xIndex *map.info.width + yIndex;
		int value = map.data[mapDataIndex];
		//ROS_INFO("X,Y (%i, %i), Map Index %i with value %i", xIndex, yIndex, mapDataIndex,value);
		if(map.data[mapDataIndex] > isEmptyValue)
		{
			//ROS_INFO("Invalid Connection %i, %i, with value %d", nodeList[startIndex].index, nodeList[endIndex].index, map.data[mapDataIndex]);
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
				////ROS_INFO("mapDataIndex %i out of bounds %i",mapDataIndex, size);
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

bool Graph::add_new_node(Node n) {
    int i;
    for(i = 0; i<nodeList.size(); i++) {
        if(nodeList[i].xindex == n.xindex && nodeList[i].yindex == n.yindex)
            return false;
    }

    nodeList.push_back(n);
    return true;
}

void Graph::draw_in_rviz(RViz_Draw *drawer)
{
    drawer->claim(visualization_msgs::Marker::LINE_LIST);
    //drawer->update_color(0, 0, 1, 1);
    for (int i = 0; i < nodeList.size(); i++)
    {
      for(int j = 0; j < nodeList[i].edgeList.size(); j++)
      {
      	int endNodeIndex = nodeList[i].edgeList[j].endNodeIndex;
		double x0 = nodeList[i].xindex;
      	double y0 = nodeList[i].yindex;
      	double x1 = nodeList[endNodeIndex].xindex;
      	double y1 = nodeList[endNodeIndex].yindex;
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
        //drawer->add_point_scale(x0, y0);
        //drawer->add_point_scale(x1, y1);
        drawer->add_node(nodeList[i]);
        drawer->add_node(nodeList[endNodeIndex]);
      }
    }
    drawer->pub();
    drawer->release();


}

void Graph::print_graph_to_console()
{
	for(int i = 0; i < nodeList.size(); i++)
	{
		int size = nodeList[i].edgeList.size();
		//ROS_INFO("Node: %i;  XY: (%i, %i); %i edges", nodeList[i].index, nodeList[i].xindex, nodeList[i].yindex, size);
		std::cout << "Node: " << nodeList[i].index << " XY (" <<  nodeList[i].xindex << "," << nodeList[i].yindex << ")  Connections : ";
		for(int j = 0; j < size; j++)
		{
			//ROS_INFO("Edge to Node %i, cost %f \n", nodeList[i].edgeList[j].endNodeIndex, nodeList[i].edgeList[j].cost);
			std::cout << nodeList[i].edgeList[j].endNodeIndex;
			if(j < size - 1) std::cout << ',';
		}
		std::cout << ";\n";
	}
}
