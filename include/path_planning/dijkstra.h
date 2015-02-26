#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include<vector>

using namespace std;
class dijkstra
{
    public:
        /** Default constructor */
        dijkstra(int numNodes);
        /** Default destructor */
        virtual ~dijkstra();

        void setGraph(float **graph);
        float** getGraph();

        void setEdgeWeight(int nodeOne, int NodeTwo, float weight);
        float getWeight(int nodeOne, int NodeTwo);

        void getShortestPath(int source, int destination, vector<int> &shortestPath);
        double getDistance(int destination) {return distance[destination];}
    protected:
    private:

        int numberOfNodes;
        float** graph;
        float* distance;
        bool* visitedSet;
        int* path;

        void runDijkstra(int src);
        int minDistance();
};

#endif // DIJKSTRA_H
