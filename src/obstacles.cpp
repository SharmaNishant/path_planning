#include <path_planning/obstacles.h>
#include <geometry_msgs/Point.h>
#include <fstream>
#include <iostream>
#include <string>
#include <cstdlib>


std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}


std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}


vector<obstacleDef> getObstaclesFromFile(char* filename)
{
    obstacleDef obstaclePoint;
    vector<obstacleDef> myObstacles;
    string line;
    ifstream myfile (filename);
    if (myfile.is_open())
    {
    while ( getline (myfile,line) )
    {
        vector<string> temp = split(line,',');
        obstaclePoint.x = atof(temp[0].c_str());
        obstaclePoint.y = atof(temp[1].c_str());
        obstaclePoint.height = atof(temp[3].c_str());
        obstaclePoint.width = atof(temp[2].c_str());
        myObstacles.push_back(obstaclePoint);
    }
    myfile.close();
    }
    else cout << "Unable to open file";
    return myObstacles;
}

obstacles::obstacles(char* filename)
{
    vector<geometry_msgs::Point> obstaclePoint;
    geometry_msgs::Point point;

    vector<obstacleDef> myObstacles = getObstaclesFromFile(filename);

    for(int i=0;i<myObstacles.size();i++)
    {
        obstaclePoint.clear();

        //first point
        point.x = myObstacles[i].x;
        point.y = myObstacles[i].y;
        point.z = 0;

        obstaclePoint.push_back(point);

        //second point
        point.x = myObstacles[i].x;
        point.y = myObstacles[i].y + myObstacles[i].height;
        point.z = 0;

        obstaclePoint.push_back(point);

        //third point
        point.x = myObstacles[i].x + myObstacles[i].width;
        point.y = myObstacles[i].y + myObstacles[i].height;
        point.z = 0;

        obstaclePoint.push_back(point);

        //fourth point
        point.x = myObstacles[i].x + myObstacles[i].width;
        point.y = myObstacles[i].y;
        point.z = 0;
        obstaclePoint.push_back(point);

        //first point again to complete the box
        point.x = myObstacles[i].x;
        point.y = myObstacles[i].y;
        point.z = 0;
        obstaclePoint.push_back(point);

        this->obstacleArray.push_back(obstaclePoint);

    }
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

vector<geometry_msgs::Point> obstacles::getObstaclePoints()
{
    vector< geometry_msgs::Point > obstaclePoints;
    geometry_msgs::Point point;
    for(int i=0 ; i< obstacleArray.size();i++)
    {
        for(int j=0; j<4; j++)
        {
            point = obstacleArray[i][j];
            obstaclePoints.push_back(point);
        }
    }
    return obstaclePoints;
}


void readSourceGoalFromFile(char* filename, float &startX, float &startY, float &goalX, float &goalY)
{
    ifstream inFile;
    inFile.open(filename);
    string line;
    if (inFile.is_open())
    {
        getline (inFile,line);
        vector<string> temp = split(line,',');
        startX = atof(temp[0].c_str());
        startY = atof(temp[1].c_str());
        //cout<< startX << " " << startY << endl;
        getline (inFile,line);
        temp = split(line,',');
        goalX = atof(temp[0].c_str());
        goalY = atof(temp[1].c_str());
       // cout<<goalX<<" "<<goalY;
        inFile.close();
    }
    else
        cout << "Unable to open file";
}
