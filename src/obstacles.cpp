#include <path_planning/obstacles.h>
#include <geometry_msgs/Point.h>


obstacles::obstacles()
{
    vector<geometry_msgs::Point> obstaclePoint;
    geometry_msgs::Point point;

    //first point
    point.x = 50;
    point.y = 50;
    point.z = 0;

    obstaclePoint.push_back(point);

    //second point
    point.x = 50;
    point.y = 70;
    point.z = 0;

    obstaclePoint.push_back(point);

    //third point
    point.x = 80;
    point.y = 70;
    point.z = 0;

    obstaclePoint.push_back(point);

    //fourth point
    point.x = 80;
    point.y = 50;
    point.z = 0;
    obstaclePoint.push_back(point);

    //first point again to complete the box
    point.x = 50;
    point.y = 50;
    point.z = 0;
    obstaclePoint.push_back(point);

    this->obstacleArray.push_back(obstaclePoint);
}

vector< vector<geometry_msgs::Point> > obstacles::getObstacleArray()
{
    return obstacleArray;
}


vector< obstacleLine > obstacles::getObstacleLines()
{
    int counter = 0;
    vector <obstacleLine> lines;
    obstacleLine line;

    for(int i=0 ; i< obstacleArray.size();i++)
    {

        for(int j=1; j<5; j++)
        {
            line.id = counter++;
            line.point[0] = obstacleArray[i][j-1];
            line.point[1] = obstacleArray[i][j];
            lines.push_back(line);
        }
    }
    return lines;
}

