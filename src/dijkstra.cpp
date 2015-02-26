#include "path_planning/dijkstra.h"
#include "limits.h"

dijkstra::dijkstra(int numNodes)
{
    this->numberOfNodes = numNodes;
    this->graph = new float*[this->numberOfNodes];
    for(int i = 0; i < this->numberOfNodes; ++i) {
        graph[i] = new float[this->numberOfNodes];
    }

    this->distance    = new float[this->numberOfNodes];
    this->visitedSet  = new bool[this->numberOfNodes];
    this->path        = new int[this->numberOfNodes];

    for(int i=0; i < this->numberOfNodes; ++i)
    {
        for(int j=0; j < this->numberOfNodes; ++j)
            graph[i][j] = 0;
        this->distance[i] = INT_MAX;
        this->visitedSet[i] = false;
        this->path[i] = -1;
    }

}

dijkstra::~dijkstra()
{
    for(int i = 0; i < numberOfNodes; ++i) {
        delete [] graph[i];
    }
    delete [] graph;

    delete [] visitedSet;

    delete [] distance;

    delete [] path;
}


void dijkstra::getShortestPath(int source, int destination, vector<int> &shortestPath)
{
    this->runDijkstra(source);
    while(path[destination] != -1)
    { //If we are not at the source node
        shortestPath.insert(shortestPath.begin(),destination);
        destination = path[destination];
    }
    shortestPath.insert(shortestPath.begin(),source);
}

float dijkstra::getWeight(int nodeOne, int NodeTwo)
{
    return this->graph[nodeOne][NodeTwo];
}

void dijkstra::setGraph(float **graph)
{
    this->graph = graph;
}

float** dijkstra::getGraph()
{
    return this->graph;
}

void dijkstra::setEdgeWeight(int nodeOne, int NodeTwo, float weight)
{
    this->graph[nodeOne][NodeTwo] = weight;
}

void dijkstra::runDijkstra(int src)
{
         // Initialize all distances as INFINITE and stpSet[] as false
     for (int i = 0; i < numberOfNodes; i++)
    {
        distance[i] = INT_MAX;
        visitedSet[i] = false;
        path[i] = -1;
    }
     // Distance of source vertex from itself is always 0
     distance[src] = 0;

     // Find shortest path for all vertices
     for (int count = 0; count < numberOfNodes-1; count++)
     {
       // Pick the minimum distance vertex from the set of vertices not
       // yet processed. u is always equal to src in first iteration.
       int u = this->minDistance();

       // Mark the picked vertex as processed
       visitedSet[u] = true;

       // Update dist value of the adjacent vertices of the picked vertex.
       for (int v = 0; v < numberOfNodes; v++)
        {
         // Update dist[v] only if is not in sptSet, there is an edge from
         // u to v, and total weight of path from src to  v through u is
         // smaller than current value of dist[v]
         if (!visitedSet[v] && graph[u][v] && distance[u] != INT_MAX
                                       && distance[u]+graph[u][v] < distance[v])
            {
                distance[v] = distance[u] + graph[u][v];
                path[v] = u;
            }
            }
     }
}

int dijkstra::minDistance()
{
   float min = INT_MAX;
   int min_index;

   for (int v = 0; v < numberOfNodes; v++)
     if (visitedSet[v] == false && distance[v] <= min)
         min = distance[v], min_index = v;

   return min_index;
}

