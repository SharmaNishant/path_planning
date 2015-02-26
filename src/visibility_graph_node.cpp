#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <path_planning/obstacles.h>
#include <path_planning/dijkstra.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <time.h>

#define success false
#define running true


int sourceX =  2;
int sourceY =  2;
int goalX   = 95;
int goalY   = 95;

struct lineData
{
    int pointOneID;
    geometry_msgs::Point pointOne;
    int pointTwoID;
    geometry_msgs::Point pointTwo;
    float length;
};

void initializeMarkers(visualization_msgs::Marker &sourcePoint,
    visualization_msgs::Marker &goalPoint,
    visualization_msgs::Marker &pathMarker,
    visualization_msgs::Marker &finalPath)
{
    //init headers
	sourcePoint.header.frame_id    = goalPoint.header.frame_id    = pathMarker.header.frame_id    = finalPath.header.frame_id    = "path_planner";
	sourcePoint.header.stamp       = goalPoint.header.stamp       = pathMarker.header.stamp       = finalPath.header.stamp       = ros::Time::now();
	sourcePoint.ns                 = goalPoint.ns                 = pathMarker.ns                 = finalPath.ns                 = "path_planner";
	sourcePoint.action             = goalPoint.action             = pathMarker.action             = finalPath.action             = visualization_msgs::Marker::ADD;
	sourcePoint.pose.orientation.w = goalPoint.pose.orientation.w = pathMarker.pose.orientation.w = finalPath.pose.orientation.w = 1.0;

    //setting id for each marker
    sourcePoint.id    = 0;
	goalPoint.id      = 1;
	pathMarker.id  = 3;
    finalPath.id      = 4;

	//defining types
	pathMarker.type                                    = visualization_msgs::Marker::LINE_LIST;
	finalPath.type                                        = visualization_msgs::Marker::LINE_STRIP;
	sourcePoint.type  = visualization_msgs::Marker::POINTS;
	goalPoint.type = visualization_msgs::Marker::SPHERE;

	//setting scale
	pathMarker.scale.x = 0.2;
	finalPath.scale.x     = 1;
	sourcePoint.scale.x   = goalPoint.scale.x = 2;
    sourcePoint.scale.y   = goalPoint.scale.y = 2;
    sourcePoint.scale.z   = goalPoint.scale.z = 1;

    //assigning colors
	sourcePoint.color.r   = 1.0f;
	goalPoint.color.g     = 1.0f;

	pathMarker.color.r = 0.8f;
	pathMarker.color.g = 0.4f;

	finalPath.color.r = 0.2f;
	finalPath.color.g = 0.2f;
	finalPath.color.b = 1.0f;

	sourcePoint.color.a = goalPoint.color.a = pathMarker.color.a = finalPath.color.a = 1.0f;
}

vector< vector<geometry_msgs::Point> > getObstacles(obstacles &obst)
{
    return obst.getObstacleArray();
}

int checkOtherLines(vector < obstacleLine > obstacleLines, double newX, double newY, int &index, int boundaryID)
{
    for(int i=0; i< obstacleLines.size(); i++)
    {
        if(i == boundaryID)
        {
            continue;
        }
        if(obstacleLines[i].point[0].x == newX && obstacleLines[i].point[0].y == newY)
        {
            index = 1;
            return i;

        }
        if(obstacleLines[i].point[1].x == newX && obstacleLines[i].point[1].y == newY)
        {
         //   ROS_INFO("check other lines, %d || %d", 0, i);
            index = 0;
            return i;
        }
    }
    index = 0;
    return -1;
}

bool get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y,
    float p2_x, float p2_y, float p3_x, float p3_y)
{

    float x,y;
    float s1_x, s1_y, s2_x, s2_y;
    s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

    float s, t;
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
    {
        // Collision detected
        x = p0_x + (t * s1_x);
        y = p0_y + (t * s1_y);

        if(x == p2_x && y == p2_y) return false;
        if(x == p3_x && y == p3_y) return false;
        return true;
    }

    return false; // No collision
}

void getLength(lineData &line)
{
    line.length = sqrt(pow(line.pointOne.x - line.pointTwo.x,2)+pow(line.pointOne.y - line.pointTwo.y,2));
}

vector< lineData > generateLines(vector<geometry_msgs::Point> &obstaclePoints, int sourceX, int sourceY, int goalX, int goalY)
{
    vector< lineData > lines;
    lineData line;

    geometry_msgs::Point tempPoint;
    tempPoint.x = goalX;
    tempPoint.y = goalY;
    obstaclePoints.insert(obstaclePoints.begin(),tempPoint);
    tempPoint.x = sourceX;
    tempPoint.y = sourceY;
    obstaclePoints.insert(obstaclePoints.begin(),tempPoint);

    for(int i=0; i< obstaclePoints.size(); i++)
    {
        for(int j=0; j< obstaclePoints.size(); j++)
        {
            if((i == 0 && j == 1) || (i == 1 && j == 0)) continue;
            if(i==j) continue;
            line.pointOneID = i;
            line.pointOne = obstaclePoints[i];
            line.pointTwoID = j;
            line.pointTwo = obstaclePoints[j];
            getLength(line);
            lines.push_back(line);
        }
    }
    return lines;
}

bool checkOverlap(lineData line, vector< obstacleLine > &obstacleLines)
{
    for(int i=0; i<obstacleLines.size();i++)
    {
        //if point one  same check for two
        if(obstacleLines[i].point[0].x == line.pointOne.x && obstacleLines[i].point[0].y == line.pointOne.y)
            if(obstacleLines[i].point[1].x == line.pointTwo.x && obstacleLines[i].point[1].y == line.pointTwo.y)
                return true;

        //if point two  same check for one
        if(obstacleLines[i].point[0].x == line.pointTwo.x && obstacleLines[i].point[0].y == line.pointTwo.y)
            if(obstacleLines[i].point[1].x == line.pointOne.x && obstacleLines[i].point[1].y == line.pointOne.y)
                return true;
    }
    return false;
}

bool checkObstacleDiagonal(lineData line, vector< vector<geometry_msgs::Point> > &obstacleArray)
{

    for(int i=0; i<obstacleArray.size();i++)
    {
        //if point one  same check for two
        if(obstacleArray[i][0].x == line.pointOne.x && obstacleArray[i][0].y == line.pointOne.y)
            if(obstacleArray[i][2].x == line.pointTwo.x && obstacleArray[i][2].y == line.pointTwo.y)
                return true;

        //if point two  same check for one
        if(obstacleArray[i][1].x == line.pointTwo.x && obstacleArray[i][1].y == line.pointTwo.y)
            if(obstacleArray[i][3].x == line.pointOne.x && obstacleArray[i][3].y == line.pointOne.y)
                return true;

        //if point one  same check for two
        if(obstacleArray[i][2].x == line.pointOne.x && obstacleArray[i][2].y == line.pointOne.y)
            if(obstacleArray[i][0].x == line.pointTwo.x && obstacleArray[i][0].y == line.pointTwo.y)
                return true;

        //if point two  same check for one
        if(obstacleArray[i][3].x == line.pointTwo.x && obstacleArray[i][3].y == line.pointTwo.y)
            if(obstacleArray[i][1].x == line.pointOne.x && obstacleArray[i][1].y == line.pointOne.y)
                return true;
    }

    return false;
}

bool checkIntersection(lineData line, vector< obstacleLine > &obstacleLines)
{
    for(int i=0; i<obstacleLines.size();i++)
    {
        char result = get_line_intersection(obstacleLines[i].point[0].x, obstacleLines[i].point[0].y, obstacleLines[i].point[1].x, obstacleLines[i].point[1].y,
                            line.pointOne.x, line.pointOne.y, line.pointTwo.x, line.pointTwo.y);
        if(result == true) return true;
    }
    return false;
}

void removeExtraLines(vector< lineData > &lines, vector< obstacleLine > &obstacleLines, vector< vector<geometry_msgs::Point> > &obstacleArray)
{
    vector< lineData > newLines;
    int i=0;
    while(i<lines.size())
    {
        if(checkOverlap(lines[i],obstacleLines))
        {
            //cout<<"Overlap"<<endl;
            //lines.erase(lines.begin()+i);
            newLines.push_back(lines[i]);
            i++;
            continue;
        }
        if(checkObstacleDiagonal(lines[i],obstacleArray))
        {
            //cout<<"diagonal"<<endl;
            //lines.erase(lines.begin()+i);
            i++;
            continue;
        }
        if(checkIntersection(lines[i],obstacleLines))
        {
            //cout<<"intersect"<<endl;
            //lines.erase(lines.begin()+i);
            i++;
            continue;
        }
        newLines.push_back(lines[i]);
        i++;
    }
    lines = newLines;
}

void generateGraph(vector< lineData > &lines, dijkstra &myDijkstra)
{
    for(int i=0; i< lines.size();i++)
    {
        //cout<< "line "<<i<<" "<<lines[i].pointOneID<<" - "<<lines[i].pointTwoID<<endl;
        myDijkstra.setEdgeWeight(lines[i].pointOneID,lines[i].pointTwoID,lines[i].length);
    }
}


void allPathsMarker(vector<lineData> &lines, visualization_msgs::Marker &vgPathsMarker)
{
    vgPathsMarker.points.clear();
    for(int i=0;i< lines.size(); i++)
    {
        vgPathsMarker.points.push_back(lines[i].pointOne);
        vgPathsMarker.points.push_back(lines[i].pointTwo);
    }
}

void finalPathMarker(vector<geometry_msgs::Point> &obstaclePoints, vector<int> &path, visualization_msgs::Marker &finalPathMarker)
{
    finalPathMarker.points.clear();
    for(int i=0; i< path.size(); i++)
    {
        //cout<<path[i]<<endl;
        finalPathMarker.points.push_back(obstaclePoints[path[i]]);
    }
}

int main(int argc,char** argv)
{
    //initializing ROS
    ros::init(argc,argv,"vg_node");
	ros::NodeHandle n;

	//defining Publisher
	ros::Publisher vg_publisher = n.advertise<visualization_msgs::Marker>("path_planner_vg",1);

	//defining markers
    visualization_msgs::Marker sourcePoint;
    visualization_msgs::Marker goalPoint;
    visualization_msgs::Marker vgPathsMarker;
    visualization_msgs::Marker finalPath;

    initializeMarkers(sourcePoint, goalPoint, vgPathsMarker, finalPath);

    //setting source and goal
    sourcePoint.pose.position.x = 2;
    sourcePoint.pose.position.y = 2;

    goalPoint.pose.position.x = 95;
    goalPoint.pose.position.y = 95;

    srand (time(NULL));

    obstacles obst;

    vector< lineData > lines;

    vector< vector<geometry_msgs::Point> >  obstacleList = getObstacles(obst);
    vector< obstacleLine > obstacleLines = obst.getObstacleLines();
    vector<geometry_msgs::Point> obstaclePoints = obst.getObstaclePoints();

    vector<int> path;
    //algprithm here
    double pathLength=0;

    ros::Time startTime = ros::Time::now();

    lines = generateLines(obstaclePoints,sourceX,sourceY,goalX,goalY);

    removeExtraLines(lines, obstacleLines,obstacleList);

    dijkstra myDijkstra(obstaclePoints.size());

    generateGraph(lines,myDijkstra);

    myDijkstra.getShortestPath(0,1,path);

    ros::Time endTime = ros::Time::now();
    ROS_INFO("Total Time %d", endTime.nsec - startTime.nsec);
    allPathsMarker(lines,vgPathsMarker);
    finalPathMarker(obstaclePoints,path,finalPath);
    ros::Duration(1).sleep();
    vg_publisher.publish(sourcePoint);
    vg_publisher.publish(goalPoint);
    vg_publisher.publish(vgPathsMarker);
    ROS_INFO("Total Path %ld", vgPathsMarker.points.size());
    vg_publisher.publish(finalPath);
    ros::spinOnce();
    ros::Duration(0.01).sleep();
	return 1;
}
