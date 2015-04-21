#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <path_planning/rrt.h>
#include <path_planning/obstacles.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <time.h>
#include <fstream>

#define success false
#define running true

using namespace rrt;

bool status = running;
    float sourceX,sourceY, goalX, goalY;

double finalPathDistance = 0;

int kValue = 3;

void initializeMarkers(visualization_msgs::Marker &sourcePoint,
    visualization_msgs::Marker &goalPoint,
    visualization_msgs::Marker &randomPoint,
    visualization_msgs::Marker &rrtTreeMarker,
    visualization_msgs::Marker &finalPath)
{
    //init headers
	sourcePoint.header.frame_id    = goalPoint.header.frame_id    = randomPoint.header.frame_id    = rrtTreeMarker.header.frame_id    = finalPath.header.frame_id    = "path_planner";
	sourcePoint.header.stamp       = goalPoint.header.stamp       = randomPoint.header.stamp       = rrtTreeMarker.header.stamp       = finalPath.header.stamp       = ros::Time::now();
	sourcePoint.ns                 = goalPoint.ns                 = randomPoint.ns                 = rrtTreeMarker.ns                 = finalPath.ns                 = "path_planner";
	sourcePoint.action             = goalPoint.action             = randomPoint.action             = rrtTreeMarker.action             = finalPath.action             = visualization_msgs::Marker::ADD;
	sourcePoint.pose.orientation.w = goalPoint.pose.orientation.w = randomPoint.pose.orientation.w = rrtTreeMarker.pose.orientation.w = finalPath.pose.orientation.w = 1.0;

    //setting id for each marker
    sourcePoint.id    = 0;
	goalPoint.id      = 1;
	randomPoint.id    = 2;
	rrtTreeMarker.id  = 3;
    finalPath.id      = 4;

	//defining types
	rrtTreeMarker.type                                    = visualization_msgs::Marker::LINE_LIST;
	finalPath.type                                        = visualization_msgs::Marker::LINE_STRIP;
	sourcePoint.type  = goalPoint.type = randomPoint.type = visualization_msgs::Marker::SPHERE;

	//setting scale
	rrtTreeMarker.scale.x = 0.2;
	finalPath.scale.x     = 1;
	sourcePoint.scale.x   = goalPoint.scale.x = randomPoint.scale.x = 2;
    sourcePoint.scale.y   = goalPoint.scale.y = randomPoint.scale.y = 2;
    sourcePoint.scale.z   = goalPoint.scale.z = randomPoint.scale.z = 1;

    //assigning colors
	sourcePoint.color.r   = 1.0f;
	goalPoint.color.g     = 1.0f;
    randomPoint.color.b   = 1.0f;

	rrtTreeMarker.color.r = 0.8f;
	rrtTreeMarker.color.g = 0.4f;

	finalPath.color.r = 0.2f;
	finalPath.color.g = 0.2f;
	finalPath.color.b = 1.0f;

	sourcePoint.color.a = goalPoint.color.a = randomPoint.color.a = rrtTreeMarker.color.a = finalPath.color.a = 1.0f;
}

vector< vector<geometry_msgs::Point> > getObstacles(char* filename)
{
    obstacles obst(filename);
    return obst.getObstacleArray();
}

void addBranchtoRRTTree(visualization_msgs::Marker &rrtTreeMarker, RRT::rrtNode &tempNode, RRT &myRRT)
{

    geometry_msgs::Point point;

    point.x = tempNode.posX;
    point.y = tempNode.posY;
    point.z = 0;
    rrtTreeMarker.points.push_back(point);

    RRT::rrtNode parentNode = myRRT.getParent(tempNode.nodeID);

    point.x = parentNode.posX;
    point.y = parentNode.posY;
    point.z = 0;

    rrtTreeMarker.points.push_back(point);
}

bool checkIfInsideBoundary(RRT::rrtNode &tempNode)
{
    if(tempNode.posX < 0 || tempNode.posY < 0  || tempNode.posX > 100 || tempNode.posY > 100 ) return false;
    else return true;
}

bool checkIfOutsideObstacles(vector< vector<geometry_msgs::Point> > &obstArray, RRT::rrtNode &tempNode)
{
    double AB, AD, AMAB, AMAD;

    for(int i=0; i<obstArray.size(); i++)
    {
        AB = (pow(obstArray[i][0].x - obstArray[i][1].x,2) + pow(obstArray[i][0].y - obstArray[i][1].y,2));
        AD = (pow(obstArray[i][0].x - obstArray[i][3].x,2) + pow(obstArray[i][0].y - obstArray[i][3].y,2));
        AMAB = (((tempNode.posX - obstArray[i][0].x) * (obstArray[i][1].x - obstArray[i][0].x)) + (( tempNode.posY - obstArray[i][0].y) * (obstArray[i][1].y - obstArray[i][0].y)));
        AMAD = (((tempNode.posX - obstArray[i][0].x) * (obstArray[i][3].x - obstArray[i][0].x)) + (( tempNode.posY - obstArray[i][0].y) * (obstArray[i][3].y - obstArray[i][0].y)));
         //(0<AM⋅AB<AB⋅AB)∧(0<AM⋅AD<AD⋅AD)
        if((0 < AMAB) && (AMAB < AB) && (0 < AMAD) && (AMAD < AD))
        {
            return false;
        }
    }
    return true;
}

void generateTempPoint(RRT::rrtNode &tempNode)
{
    int x = rand() % 100 + 1;
    int y = rand() % 100 + 1;
    //std::cout<<"Random X: "<<x <<endl<<"Random Y: "<<y<<endl;
    tempNode.posX = x;
    tempNode.posY = y;
}

bool checkNodetoGoal(int X, int Y, RRT::rrtNode &tempNode)
{
    double distance = sqrt(pow(X-tempNode.posX,2)+pow(Y-tempNode.posY,2));
    if(distance < 1)
    {
        return true;
    }
    return false;
}

bool addNewPointtoRRT(RRT &myRRT, RRT::rrtNode randPointNode, int rrtStepSize, vector< vector<geometry_msgs::Point> > &obstArray, visualization_msgs::Marker &rrtTreeMarker, vector< vector<int> > &rrtPaths)
{
    //ROS_INFO("Entering getKnearestNodes");
    vector<int> nearestNodeID = myRRT.getKNearestNodesID(randPointNode.posX,randPointNode.posY, kValue);
   // ROS_INFO("Exiting getKnearestNodes");
    int i = 0;
    RRT::rrtNode tempNode;
    vector<int> path;
    bool nodeToGoal = false;
    while(i < kValue)
    {
        if(nearestNodeID[i] == -1) break;
        RRT::rrtNode nearestNode = myRRT.getNode(nearestNodeID[i]);

        double theta = atan2(randPointNode.posY - nearestNode.posY,randPointNode.posX - nearestNode.posX);

        tempNode.posX = nearestNode.posX + (rrtStepSize * cos(theta));
        tempNode.posY = nearestNode.posY + (rrtStepSize * sin(theta));

        if(checkIfInsideBoundary(tempNode) && checkIfOutsideObstacles(obstArray,tempNode))
        {
            tempNode.parentID = nearestNodeID[i];
            tempNode.nodeID = myRRT.getTreeSize();
            myRRT.addNewNode(tempNode);

            addBranchtoRRTTree(rrtTreeMarker,tempNode,myRRT);
           // std::cout<<"tempnode printed"<<endl;
            nodeToGoal = checkNodetoGoal(goalX, goalY,tempNode);
            if(nodeToGoal)
            {
                path = myRRT.getRootToEndPath(tempNode.nodeID);
                rrtPaths.push_back(path);
            }
        }
        i++;
        if(i >= nearestNodeID.size()) break;
     }
    return true;
}

void setFinalPathData(vector< vector<int> > &rrtPaths, RRT &myRRT, int i, visualization_msgs::Marker &finalpath, int goalX, int goalY)
{
    RRT::rrtNode tempNode;
    geometry_msgs::Point point;
    for(int j=0; j<rrtPaths[i].size();j++)
    {
        tempNode = myRRT.getNode(rrtPaths[i][j]);

        point.x = tempNode.posX;
        point.y = tempNode.posY;
        point.z = 0;

        finalpath.points.push_back(point);
    }

    point.x = goalX;
    point.y = goalY;
    finalpath.points.push_back(point);
}

bool getLineIntersections(float pointOneX, float pointOneY, float pointTwoX, float pointTwoY, vector < obstacleLine > &obstacleLines)
{
    float p0_x = pointOneX;
    float p0_y = pointOneY;
    float p1_x = pointTwoX;
    float p1_y = pointTwoY;

    float p2_x;
    float p2_y;
    float p3_x;
    float p3_y;

    float i_x,i_y;


    for(int i=0; i< obstacleLines.size();i++)
    {
        p2_x = obstacleLines[i].point[0].x;
        p2_y = obstacleLines[i].point[0].y;
        p3_x = obstacleLines[i].point[1].x;
        p3_y = obstacleLines[i].point[1].y;

        float s1_x, s1_y, s2_x, s2_y;
        s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
        s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

        float s, t;
        s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
        t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

        if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
        {
            // Collision detected
            i_x = p0_x + (t * s1_x);
            i_y = p0_y + (t * s1_y);

            if(i_x == p0_x && i_y == p0_y) continue;
            if(i_x == p1_x && i_y == p1_y) continue;

            return true;
        }
    }
    return false; // No collision
}

void pruningFinalPath(vector < obstacleLine > &obstacleLines, visualization_msgs::Marker &finalPath)
{
    vector< geometry_msgs::Point > prunedPath;
    finalPathDistance = 0;
    prunedPath.push_back(finalPath.points[0]);

    bool intersectionResult = false;

    for(int i=1;i<finalPath.points.size();i++)
    {
        intersectionResult = getLineIntersections(prunedPath[prunedPath.size() - 1].x,prunedPath[prunedPath.size() - 1].y, finalPath.points[i].x, finalPath.points[i].y,obstacleLines);
        if(intersectionResult == true) //intersection detected
        {
            prunedPath.push_back(finalPath.points[i-1]);
            finalPathDistance += sqrt(pow(prunedPath[prunedPath.size() - 1].x - prunedPath[prunedPath.size() - 2].x,2)+pow(prunedPath[prunedPath.size() - 1].y - prunedPath[prunedPath.size() - 2].y,2));
        }
    }
    prunedPath.push_back(finalPath.points[finalPath.points.size()-1]);
    finalPathDistance += sqrt(pow(prunedPath[prunedPath.size() - 1].x - prunedPath[prunedPath.size() - 2].x,2)+pow(prunedPath[prunedPath.size() - 1].y - prunedPath[prunedPath.size() - 2].y,2));
    finalPath.points = prunedPath;
   // ROS_INFO("Cost = %f",distance);
}


int main(int argc,char** argv)
{
    //initializing ROS
    ros::init(argc,argv,"rrt_node");
	ros::NodeHandle n;

	//defining Publisher
	ros::Publisher rrt_publisher = n.advertise<visualization_msgs::Marker>("path_planner_rrt",1);

	//defining markers
    visualization_msgs::Marker sourcePoint;
    visualization_msgs::Marker goalPoint;
    visualization_msgs::Marker randomPoint;
    visualization_msgs::Marker rrtTreeMarker;
    visualization_msgs::Marker finalPath;

    initializeMarkers(sourcePoint, goalPoint, randomPoint, rrtTreeMarker, finalPath);

    readSourceGoalFromFile(argv[2],sourceX,sourceY,goalX,goalY);
    //setting source and goal
    sourcePoint.pose.position.x = sourceX;
    sourcePoint.pose.position.y = sourceY;

    goalPoint.pose.position.x = goalX;
    goalPoint.pose.position.y = goalY;

    srand (time(NULL));

    //initialize rrt specific variables
    //initializing rrtTree
    RRT myRRT(sourceX,sourceY);

    int rrtStepSize = 1;

    vector< vector<int> > rrtPaths;

    int rrtPathLimit = 1;

   //// cout<<"RRT min paths"<<endl;
   //// cin>>rrtPathLimit;
   // cout<<"k value"<<endl;
    //cin>>kValue;

    double shortestPathLength = 9999;
    int shortestPath = -1;

    RRT::rrtNode tempNode;
    vector<RRT::rrtNode> tempNodeList;
    obstacles obstacle(argv[1]);
    vector < obstacleLine > obstacleLines = obstacle.getObstacleLines();
    vector< vector<geometry_msgs::Point> >  obstacleList = getObstacles(argv[1]);

    bool addNodeResult = false, nodeToGoal = false;

    ros::Time time = ros::Time::now();

    while(ros::ok() && status)
    {
        if(rrtPaths.size() < rrtPathLimit)
        {

            generateTempPoint(tempNode);

            //std::cout<<"tempnode generated"<<endl;
           // ROS_INFO("Entering add point to RRT");
            addNewPointtoRRT(myRRT,tempNode,rrtStepSize,obstacleList,rrtTreeMarker,rrtPaths);
            //ROS_INFO("Exiting add point to RRT");
        }
        else //if(rrtPaths.size() >= rrtPathLimit)
        {
            status = success;
            for(int i=0; i<rrtPaths.size();i++)
            {
                if(rrtPaths[i].size() < shortestPath)
                {
                    shortestPath = i;
                    shortestPathLength = rrtPaths[i].size();
                }
            }
            setFinalPathData(rrtPaths, myRRT, shortestPath, finalPath, goalX, goalY);
            pruningFinalPath(obstacleLines, finalPath);
            break;
        }
            //rrt_publisher.publish(rrtTreeMarker);
            //ros::Duration(0.1).sleep();
    }
    ros::Time endTime = ros::Time::now();
    int nsec = endTime.nsec - time.nsec;
    int sec = endTime.sec - time.sec;
    if(nsec < 0)
    {
        sec -= 1;
        nsec += 1000000000;
    }
    ROS_INFO("End, Total Time = %d, %d", sec, nsec);
    ROS_INFO("PATH Size - %f", shortestPathLength);
double mainTime = sec + (nsec / 1000000000.0);

    time = ros::Time::now();
    pruningFinalPath(obstacleLines,finalPath);

    endTime = ros::Time::now();
     nsec = endTime.nsec - time.nsec;
     sec = endTime.sec - time.sec;
    if(nsec < 0)
    {
        sec -= 1;
        nsec += 1000000000;
    }
    ROS_INFO("Pruning Time = %d, %d", sec, nsec);
    ROS_INFO("COST %f", finalPathDistance);
    double prunTime = sec + (nsec / 1000000000.0);

    rrt_publisher.publish(sourcePoint);
    rrt_publisher.publish(goalPoint);
    rrt_publisher.publish(rrtTreeMarker);
    ROS_INFO("End, Total branches = %ld", rrtTreeMarker.points.size());
    rrt_publisher.publish(finalPath);
    ros::spinOnce();
    ros::Duration(1).sleep();
    ofstream logFile;
    logFile.open("rrtsLog.txt",ofstream::app);
    logFile << shortestPathLength << ","<< finalPathDistance << ","<< mainTime << "," << prunTime << endl;
    logFile.close();	return 1;
}

