#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <path_planning/obstacles.h>
#include <path_planning/Bug.h>
#include <path_planning/Vec2.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <vector>
#include <cmath>

#define debug

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

void displayBugs(vector<Bug> &bugList, visualization_msgs::Marker &sourcePoint, ros::Publisher &bug_publisher)
{
    vector < geometry_msgs::Point> points;
    geometry_msgs::Point point;

    point.x = sourceX;
    point.y = sourceY;
    point.z = 0;
    points.push_back(point);

    for(int i=0;i<bugList.size();i++)
    {
        point.x = bugList[i].getX();
        point.y = bugList[i].getY();
        point.z = 0;
        points.push_back(point);
//#ifdef debug
//    ROS_INFO("bug %d is at %f || %f", i+1, bugList[i].getX(), bugList[i].getY());
//#endif
    }
    sourcePoint.points = points;
    bug_publisher.publish(sourcePoint);
}

void displayPaths(vector< geometry_msgs::Point > &points,visualization_msgs::Marker &pathMarker,ros::Publisher &bug_publisher)
{
    pathMarker.points = points;
    bug_publisher.publish(pathMarker);
}


void displayFinalPath(LocationArray &locationList,visualization_msgs::Marker &finalPath,ros::Publisher &bug_publisher)
{
    double min_path = 9999;
    double path_length;
    int minPathId;

    vector< geometry_msgs::Point > points;
    geometry_msgs::Point point;
    vector<location> path;
    location temp;
    int i=1;
    vector<location> loc = locationList.getLocationList();
    while(i != 0)
    {
        //ROS_INFO("curr node = %d || %d",loc[i].getCurrNode(), loc[i].getPrevNode());

        point.x = loc[i].getX();
        point.y = loc[i].getY();
        points.push_back(point);
        i = loc[i].getPrevNode();
//      loc = locationList.getElement(i);
        //ros::Duration(1).sleep();
        //i++;
    }

    i=0;
    while(i < loc.size())
    {
        ROS_INFO("curr node = %d || %d || %d", loc[i].getPrevNode(), loc[i].getCurrNode(), loc[i].getNextNode());
        i++;
    }

    ros::Duration(1).sleep();
    point.x = sourceX;
    point.y = sourceY;
    points.push_back(point);
   //ROS_INFO("Min Path %d -> Length = %f",minPathId,bugs[minPathId].getPathLength());
//
//    path = bugs[minPathId].getPath();
//    for(int j=1; j<bugs[minPathId].getPathLength();j++)
//    {
//        point.x = path[j-1].x;
//        point.y = path[j-1].y;
//        //ROS_INFO("Min Path index %d -> points = %f || %f",j,path[j-1].x,path[j-1].y);
//        points.push_back(point);
//        point.x = path[j].x;
//        point.y = path[j].y;
//        points.push_back(point);
//    }
    finalPath.points = points;
    bug_publisher.publish(finalPath);
}


void boundaryFollow(vector<Bug> &bugList,int i, vector < obstacleLine > &obstacleLines, vector< vector<geometry_msgs::Point> > &obstArray,
            vector< geometry_msgs::Point > &pathList, LocationArray &locationList, bool &killFlag)
{
 //   ROS_INFO("enter boundary follow");
    geometry_msgs::Point point;
    killFlag = false;
    location loc;
    if(bugList[i].checkInsideObstacles(obstArray, goalX, goalY))
    {
        int index;
        int line = bugList[i].checkIfOnOtherLines(obstacleLines, index);


        bugList[i].setX(obstacleLines[line].point[index].x);
        bugList[i].setY(obstacleLines[line].point[index].y);
        bugList[i].setBoundaryID(line);
        bugList[i].addDistance(bugList[i].getEuclideanDistance(bugList[i].getX(),bugList[i].getY(),bugList[i].getLastX(),bugList[i].getLastY()));

        /** add to path **/
        loc.setX(bugList[i].getX());
        loc.setY(bugList[i].getY());
        loc.setDistance(bugList[i].getDistance());
        loc.setPrevNode(bugList[i].getLastPointID());
        loc.addBugtoList(bugList[i].getID());

        /** adding to visulization list **/
        point.x = bugList[i].getX();
        point.y = bugList[i].getY();
        point.z = 0;
        pathList.push_back(point);
        point.x = bugList[i].getLastX();
        point.y = bugList[i].getLastY();
        point.z = 0;
        pathList.push_back(point);

        int locID = locationList.searchLocation(bugList[i].getX(), bugList[i].getY());
        if(locID != -1)
        {
            float prevDistanceToPoint = locationList.getDistance(locID);
            if(prevDistanceToPoint <= loc.getDistance() && prevDistanceToPoint != -1)
            {
                killFlag = true;
            }
            else
            {
                if(locID == loc.getPrevNode())
                {
                    locationList.updateLocation(locID, loc, true);
                }
                else
                {
                    locationList.updateLocation(locID, loc, false);
                }
                bugList[i].setLastPointID(locID);
            //int tempID = locationList.addLocationToList(loc);
            //bugList[i].setLastPointID(tempID);
            }
        }
        else
        {
            int tempID = locationList.addLocationToList(loc);
            bugList[i].setLastPointID(tempID);
        }
    }
    else
    {
   //     ROS_INFO("change state");
        bugList[i].setState(movingToGoal);
    }
}

void moveToGoal(vector<Bug> &bugList, int i, vector < obstacleLine > &obstacleLines, vector< vector<geometry_msgs::Point> > &obstArray,
            vector< geometry_msgs::Point > &pathList, LocationArray &locationList, bool &killFlag)
{
    geometry_msgs::Point point;
    killFlag = false;
    location loc;
    vector<intersectingPoint> lineIds;
    lineIds = bugList[i].getLineIntersections(goalX, goalY, obstacleLines);

//#ifdef debug
//    ROS_INFO("enter Move To Goal");
//#endif // debug

    /** routine for getting nearest line **/
    int lineId;
    int intersectionPointID;
    bool lineFlag = false;
    float minDist = 9999;
    float dist;
    for(int j=0;j < lineIds.size();j++)
    {
        //if((bugList[i].getX() == lineIds[j].x) && (bugList[i].getY() == lineIds[j].y)) continue;

        dist = bugList[i].getEuclideanDistance(bugList[i].getX(),bugList[i].getY(), lineIds[j].x,lineIds[j].y);

        if(dist<minDist)
        {
            minDist = dist;
            lineId = lineIds[j].ID;
            intersectionPointID = j;
            lineFlag = true;
        }
    }

//#ifdef debug
//    ROS_INFO("nearest interesting line is calculated");
//#endif

    /** if there is a line in the way **/
    if(lineFlag)
    {

        bugList[i].setX(lineIds[intersectionPointID].x);
        bugList[i].setY(lineIds[intersectionPointID].y);
        bugList[i].addDistance(minDist);

                point.x = bugList[i].getX();
        point.y = bugList[i].getY();
        point.z = 0;
        pathList.push_back(point);
        point.x = bugList[i].getLastX();
        point.y = bugList[i].getLastY();
        point.z = 0;
        pathList.push_back(point);

        location loc;
        loc.setX(lineIds[intersectionPointID].x);
        loc.setY(lineIds[intersectionPointID].y);
        loc.setDistance(bugList[i].getDistance());
        loc.setPrevNode(bugList[i].getLastPointID());
        loc.addBugtoList(bugList[i].getID());

//#ifdef debug
//    ROS_INFO("starting adding point on obstacle boundary");
//#endif

        /*** adding intersection point ***/
        int locID = locationList.searchLocation(lineIds[intersectionPointID].x,lineIds[intersectionPointID].y);
        if(locID != -1)
        {
            float prevDistanceToPoint = locationList.getDistance(locID);
            if(prevDistanceToPoint <= loc.getDistance() && prevDistanceToPoint != -1)
            {
//#ifdef debug
//                ROS_INFO("previous path exists with lesser distance");
//#endif
                killFlag = true;
                return;
            }
            else
            {
                //if(!(locID == loc.getPrevNode()))
//                {
//                    locationList.updateLocation(locID, loc);
//                    bugList[i].setLastPointID(locID);
//                }
                if(locID == loc.getPrevNode())
                {
                    locationList.updateLocation(locID, loc, true);
                }
                else
                {
                    locationList.updateLocation(locID, loc, false);
                }
                bugList[i].setLastPointID(locID);
            //int tempID = locationList.addLocationToList(loc);
            //bugList[i].setLastPointID(tempID);
            }
        }
        else
        {
            int tempID = locationList.addLocationToList(loc);
            bugList[i].setLastPointID(tempID);
        }

//#ifdef debug
//    ROS_INFO("done adding point on obstacle boundary");
//#endif
        /** changing state **/
        bugList[i].setBoundaryID(lineId);
        bugList[i].setState(boundaryFollowing);

        Bug newTempBug(boundaryFollowing, lineIds[intersectionPointID].x, lineIds[intersectionPointID].y, bugList[i].getDistance(), bugList[i].getLastPointID(), bugList[i].getBoundaryID());
//        bool samePoint = false;
//        if(bugList[i].getLastX() == obstacleLines[lineId].point[0].x && bugList[i].getLastY() == obstacleLines[lineId].point[0].y)
//        samePoint = true;

        bugList[i].setY(obstacleLines[lineId].point[1].y);
        bugList[i].setX(obstacleLines[lineId].point[1].x);
       bugList[i].addDistance(bugList[i].getEuclideanDistance(bugList[i].getLastX(),bugList[i].getLastY(),obstacleLines[lineId].point[1].x,obstacleLines[lineId].point[1].y));

        point.x = bugList[i].getX();
        point.y = bugList[i].getY();
        point.z = 0;
        pathList.push_back(point);
        point.x = bugList[i].getLastX();
        point.y = bugList[i].getLastY();
        point.z = 0;
        pathList.push_back(point);

        /** add to path **/
        loc.setX(bugList[i].getX());
        loc.setY(bugList[i].getY());
        loc.setDistance(bugList[i].getDistance());
        loc.setPrevNode(bugList[i].getLastPointID());
        loc.addBugtoList(bugList[i].getID());

        locID = locationList.searchLocation(bugList[i].getX(), bugList[i].getY());
        if(locID != -1)
        {
            float prevDistanceToPoint = locationList.getDistance(locID);
            if(prevDistanceToPoint < loc.getDistance() && prevDistanceToPoint != -1)
            {
                killFlag = true;
            }
            else
            {
//                if(!(locID == loc.getPrevNode()))
//                {
//                    locationList.updateLocation(locID, loc);
//                    bugList[i].setLastPointID(locID);
//                }
                if(locID == loc.getPrevNode())
                {
                    locationList.updateLocation(locID, loc, true);
                }
                else
                {
                    locationList.updateLocation(locID, loc, false);
                }
                bugList[i].setLastPointID(locID);
//int tempID = locationList.addLocationToList(loc);
//            bugList[i].setLastPointID(tempID);
            }
        }
        else
        {
            int tempID = locationList.addLocationToList(loc);
            bugList[i].setLastPointID(tempID);
        }


        newTempBug.setBoundaryID(lineId);
//        samePoint=false;
//        if(newTempBug.getLastX() == obstacleLines[lineId].point[0].x && newTempBug.getLastY() == obstacleLines[lineId].point[0].y)
//        samePoint = true;

        newTempBug.setX(obstacleLines[lineId].point[0].x);
        newTempBug.setY(obstacleLines[lineId].point[0].y);
        float dist = newTempBug.getEuclideanDistance(newTempBug.getX(),newTempBug.getY(),obstacleLines[lineId].point[0].x,obstacleLines[lineId].point[0].y);
        newTempBug.addDistance(dist);

        point.x = newTempBug.getX();
        point.y = newTempBug.getY();
        point.z = 0;
        pathList.push_back(point);
        point.x = newTempBug.getLastX();
        point.y = newTempBug.getLastY();
        point.z = 0;
        pathList.push_back(point);

//#ifdef debug
//        ROS_INFO("adding a new bug");
//#endif // debug
        /** add to path **/
        loc.setX(newTempBug.getX());
        loc.setY(newTempBug.getY());
        loc.setDistance(newTempBug.getDistance());
        loc.setPrevNode(newTempBug.getLastPointID());
        loc.addBugtoList(newTempBug.getID());

        locID = locationList.searchLocation(newTempBug.getX(), newTempBug.getY());
        if(locID != -1)
        {
            float prevDistanceToPoint = locationList.getDistance(locID);
            if(prevDistanceToPoint < loc.getDistance() && (prevDistanceToPoint != -1))
            {
            }
            else
            {
//                if(!(locID == loc.getPrevNode()))
//                {
//                    locationList.updateLocation(locID, loc);
//                    bugList[i].setLastPointID(locID);
//                }
                if(locID == loc.getPrevNode())
                {
                    locationList.updateLocation(locID, loc, true);
                }
                else
                {
                    locationList.updateLocation(locID, loc, false);
                }
                newTempBug.setLastPointID(locID);
//            int tempID = locationList.addLocationToList(loc);
//            newTempBug.setLastPointID(tempID);
            }
        }
        else
        {
            int tempID = locationList.addLocationToList(loc);
            newTempBug.setLastPointID(tempID);
        }
//        #ifdef debug
//        ROS_INFO("saving pointers to new bug");
//        #endif // debug
        bugList.push_back(newTempBug);
    }

/** adding bug path to goal **/
    else
    {
//        #ifdef debug
//        ROS_INFO("adding goal to bug");
//        #endif // debug
        location loc;
        bugList[i].setState(reachedEnd);
        killFlag = true;

        bugList[i].addDistance(bugList[i].getEuclideanDistance(bugList[i].getX(),bugList[i].getY(),goalX,goalY));

        bugList[i].setX(goalX);
        bugList[i].setY(goalY);

        point.x = bugList[i].getX();
        point.y = bugList[i].getY();
        point.z = 0;
        pathList.push_back(point);
        point.x = bugList[i].getLastX();
        point.y = bugList[i].getLastY();
        point.z = 0;
        pathList.push_back(point);

        /** add to path **/
        loc.setX(bugList[i].getX());
        loc.setY(bugList[i].getY());
        loc.setDistance(bugList[i].getDistance());
        loc.setPrevNode(bugList[i].getLastPointID());
        loc.addBugtoList(bugList[i].getID());

        float prevDistanceToPoint = locationList.getDistance(1);
        if(prevDistanceToPoint > loc.getDistance())
        {
            locationList.updateLocation(1, loc, false);
            bugList[i].setLastPointID(1);
        }

        killFlag = true;
    }

}


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

    /** initializing bugs**/
    vector<Bug> bugList;
    Bug mainBug(movingToGoal,0,0,0,0,-1);
    bugList.push_back(mainBug);

    LocationArray locationList;
    location loc1(0,0,0,sourceX,sourceY,0,0);
    locationList.addLocationToList(loc1);

    location loc2(0,1,0,goalX,goalY,99999,0);
    locationList.addLocationToList(loc2);

    vector< geometry_msgs::Point > pathList;

    obstacles obstacle;
    vector < obstacleLine > obstacleLines = obstacle.getObstacleLines();
    vector< vector<geometry_msgs::Point> > obstacleList = obstacle.getObstacleArray();

    vector<int> idsToRemove;

    ros::Time time = ros::Time::now();
//
    while(ros::ok())
    {
       // ROS_INFO("size list %ld || %ld", bugList.size(),idsToRemove.size());
        for(int i= 0;i < bugList.size();)
        {

            bool flaged = false;
            for(int j=0;j < idsToRemove.size();j++)
            {
                if(idsToRemove[j] == bugList[i].getID())
                {
                    bugList.erase(bugList.begin()+i);
                    flaged = true;
                    break;
                }
            }
            if(flaged) continue;
            if(bugList[i].getState() == reachedEnd)
            {
                bugList.erase(bugList.begin()+i);
                continue;
            }
            i++;
        }
        idsToRemove.clear();
       //    ros::Duration(1).sleep();
        if(bugList.size() == 0)
        {
            //ROS_INFO("breaking main loop");
            break;
        }


        for(int i=0;i<bugList.size();i++)
        {
            if(bugList[i].getState() == movingToGoal)
            {
                bool killFlag;
                moveToGoal(bugList,i,obstacleLines,obstacleList,pathList,locationList, killFlag);
                if(killFlag)
                {
                    idsToRemove.push_back(bugList[i].getID());
                }
            }
            else if(bugList[i].getState() == boundaryFollowing)
            {
                bool killFlag;
                boundaryFollow(bugList,i,obstacleLines,obstacleList, pathList,locationList, killFlag);
                if(killFlag)
                {
                    idsToRemove.push_back(bugList[i].getID());
                }
            }
        }


        displayBugs(bugList,sourcePoint,bug_publisher);
        bug_publisher.publish(sourcePoint);
        ros::Duration(1).sleep();
    }
    ros::Time endTime = ros::Time::now();
    ROS_INFO("End, Total Time = %d, %d", endTime.sec - time.sec, endTime.nsec - time.nsec);
    displayBugs(bugList,sourcePoint,bug_publisher);
    bug_publisher.publish(goalPoint);
    bug_publisher.publish(sourcePoint);
    displayPaths(pathList, pathMarker, bug_publisher);
    displayFinalPath(locationList, finalPath, bug_publisher);
    ros::Duration(1).sleep();
    ros::spinOnce();
//    displayPaths();


    return 1;
}
