#include "Edge.h"
#include "Node.h"
#include <math.>

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
