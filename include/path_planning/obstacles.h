#ifndef OBSTACLES_H
#define OBSTACLES_H
#include <geometry_msgs/Point.h>
#include <vector>
#include <iostream>

using namespace std;

struct obstacleDef{
float x;
float y;
float height;
float width;
};

struct obstacleLine
{
    int id;
    geometry_msgs::Point point[2];
};

class obstacles
{
    public:
        /** Default constructor */
        obstacles(char* filename);
        /** Default destructor */
        virtual ~obstacles() {}

        vector< vector<geometry_msgs::Point> > getObstacleArray();
        vector< obstacleLine > getObstacleLines();
        vector< geometry_msgs::Point > getObstaclePoints();
    protected:
    private:
        vector< vector<geometry_msgs::Point> > obstacleArray;
};

void readSourceGoalFromFile(char* filename, float &startX, float &startY, float &goalX, float &goalY);

#endif // OBSTACLES_H
