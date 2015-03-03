#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <path_planning/obstacles.h>
#include <path_planning/Bug.h>
#include <path_planning/Vec2.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <vector>
#include <cmath>

int sourceX =  2;
int sourceY =  2;
int goalX   = 95;
int goalY   = 95;


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

void displayBugs(Buglist bugList, visualization_msgs::Marker &sourcePoint, ros::Publisher &bug_publisher)
{
    vector < geometry_msgs::Point> points;
    geometry_msgs::Point point;

    point.x = sourceX;
    point.y = sourceY;
    point.z = 0;
    points.push_back(point);

    for(int i=0;i<bugs.size();i++)
    {
        point.x = bugs[i].getPosX();
        point.y = bugs[i].getPosY();
        point.z = 0;
    }
    sourcePoint.points = points;
    bug_publisher.publish(sourcePoint);
}

/*
void displayPaths(vector<bug> bugs,visualization_msgs::Marker &pathMarker,ros::Publisher &bug_publisher)
{
    vector< geometry_msgs::Point > points;
    geometry_msgs::Point point;
    vector<location> path;
    location temp;
    for(int i=0;i<bugs.size();i++)
    {
        path = bugs[i].getPath();
        for(int j=1; j<bugs[i].getPathLength();j++)
        {
            point.x = path[j-1].x;
            point.y = path[j-1].y;
            points.push_back(point);
            point.x = path[j].x;
            point.y = path[j].y;
            points.push_back(point);
        }
    }
    pathMarker.points = points;
    bug_publisher.publish(pathMarker);
}*/

/*
void displayFinalPath(vector<bug> bugs,visualization_msgs::Marker &finalPath,ros::Publisher &bug_publisher)
{
    double min_path = 9999;
    double path_length;
    int minPathId;

    vector< geometry_msgs::Point > points;
    geometry_msgs::Point point;
    vector<location> path;
    location temp;
    for(int i=0;i<bugs.size();i++)
    {
        path = bugs[i].getPath();
        path_length = 0;
        for(int j=1; j<bugs[i].getPathLength();j++)
        {
            path_length += sqrt(pow(path[j-1].x-path[j].x,2)+pow(path[j-1].y-path[j].y,2));
        }
        //ROS_INFO("Path %d -> Length = %f",i+1,path_length);
        if(path_length < min_path)
        {
            min_path = path_length;
            minPathId = i;
        }

    }

   //ROS_INFO("Min Path %d -> Length = %f",minPathId,bugs[minPathId].getPathLength());

    path = bugs[minPathId].getPath();
    for(int j=1; j<bugs[minPathId].getPathLength();j++)
    {
        point.x = path[j-1].x;
        point.y = path[j-1].y;
        //ROS_INFO("Min Path index %d -> points = %f || %f",j,path[j-1].x,path[j-1].y);
        points.push_back(point);
        point.x = path[j].x;
        point.y = path[j].y;
        points.push_back(point);
    }
    finalPath.points = points;
    bug_publisher.publish(finalPath);
}
*/

int main(int argc, char** argv)
{
    //initializing ROS
    ros::init(argc,argv,"bug_node");
	ros::NodeHandle n;

	//defining Publisher
	ros::Publisher bug_publisher = n.advertise<visualization_msgs::Marker>("path_planner_bug",1);
    ros::Duration(1).sleep();
	//defining markers
    visualization_msgs::Marker sourcePoint;
    visualization_msgs::Marker goalPoint;
    visualization_msgs::Marker pathMarker;
    visualization_msgs::Marker finalPath;

    initializeMarkers(sourcePoint, goalPoint, pathMarker, finalPath);

    goalPoint.pose.position.x = 95;
    goalPoint.pose.position.y = 95;

    Bug* mainBug = new Bug(movingToGoal,0,0,0,-1,-1);

    BugList bugList;
    bugList.Append(mainBug);

    LocationArray locationList;

    location loc1(0,0,0,sourceX,sourceY,0,0);
    locationList.addLocationToList(loc1);

    location loc2(0,0,0,goalX,goalY,9999,0);
    locationList.addLocationToList(loc2);


    obstacles obstacle;
    vector < obstacleLine > obstacleLines = obstacle.getObstacleLines();
    vector< vector<geometry_msgs::Point> > obstacleList = obstacle.getObstacleArray();

    int pointID,pointIDOne,pointIDTwo;
    bool killFlag;
    ros::Time time = ros::Time::now();
    Bug* newBug;
    while(ros::ok())
    {
        Bug* temp = bugList.head;

        while(temp != NULL)
        {
            temp->moveBug(goalX,goalY,obstacleLines,obstacleList,pointID,pointIDOne,pointIDTwo,locationList,killFlag,newBug);

            if(newBug != NULL) bugList.Append(newBug);
            if(killFlag == true) temp->getID();



            temp = temp->getNext();
        }

    }
    ROS_INFO("End, Total Time = %d, %d", ros::Time::now().sec - time.sec, ros::Time::now().nsec - time.nsec);
//    displayBugs(bugs,sourcePoint,bug_publisher);
//    bug_publisher.publish(goalPoint);
//    displayPaths(bugs, pathMarker, bug_publisher);
//    displayFinalPath(bugs, finalPath, bug_publisher);
    ros::Duration(1).sleep();
    ros::spinOnce();
//    displayPaths();


    return 1;
}
