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

double finalPathDistance = 0;

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

    /** adding Source to visual bug list **/
    point.x = sourceX;
    point.y = sourceY;
    point.z = 0;
    points.push_back(point);

    /** adding bugs to list
    * this loop only matters if function called during run
    */
    for(int i=0;i<bugList.size();i++)
    {
        point.x = bugList[i].getX();
        point.y = bugList[i].getY();
        point.z = 0;
        points.push_back(point);
    }
    /** publish **/
    sourcePoint.points = points;
    bug_publisher.publish(sourcePoint);
}

void displayPaths(vector< geometry_msgs::Point > &points,visualization_msgs::Marker &pathMarker,ros::Publisher &bug_publisher)
{
    pathMarker.points = points;
    bug_publisher.publish(pathMarker);
}

void calcFinalPath(LocationArray &locationList,visualization_msgs::Marker &finalPath,ros::Publisher &bug_publisher)
{
    /** defining points array for final path **/
    vector< geometry_msgs::Point > points;
    geometry_msgs::Point point;

    /** calculating path by traversing back **/
    int i=1;
    vector<location> loc = locationList.getLocationList();
    ROS_INFO("Original Path Cost = %f",loc[1].getDistance());
    while(i != 0)
    {
        point.x = loc[i].getX();
        point.y = loc[i].getY();
        points.push_back(point);
        i = loc[i].getPrevNode();
    }
    /** adding Source to final path list **/
    point.x = sourceX;
    point.y = sourceY;
    points.push_back(point);

    finalPath.points = points;
//    bug_publisher.publish(finalPath);
}

void boundaryFollow(vector<Bug> &bugList,int i, vector < obstacleLine > &obstacleLines, vector< vector<geometry_msgs::Point> > &obstArray,
            vector< geometry_msgs::Point > &pathList, LocationArray &locationList, bool &killFlag)
{
    /** defining values **/
    geometry_msgs::Point point;
    killFlag = false;
    location loc;

    /** if the bug is clear to move towards goal or not **/
    /** bug is not clear to move **/
    if(bugList[i].checkInsideObstacles(obstArray, goalX, goalY))
    {
        /** check which obstacle line can be followed **/
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

    /** routine for getting nearest line **/
    int lineId;
    int intersectionPointID;
    bool lineFlag = false;
    float minDist = 9999;
    float dist;
    for(int j=0;j < lineIds.size();j++)
    {
        dist = bugList[i].getEuclideanDistance(bugList[i].getX(),bugList[i].getY(), lineIds[j].x,lineIds[j].y);

        if(dist<minDist)
        {
            minDist = dist;
            lineId = lineIds[j].ID;
            intersectionPointID = j;
            lineFlag = true;
        }
    }


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

        /*** adding intersection point ***/
        int locID = locationList.searchLocation(lineIds[intersectionPointID].x,lineIds[intersectionPointID].y);
        if(locID != -1)
        {
            float prevDistanceToPoint = locationList.getDistance(locID);
            if(prevDistanceToPoint <= loc.getDistance() && prevDistanceToPoint != -1)
            {
                killFlag = true;
                return;
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
            }
        }
        else
        {
            int tempID = locationList.addLocationToList(loc);
            bugList[i].setLastPointID(tempID);
        }

        /** changing state **/
        bugList[i].setBoundaryID(lineId);
        bugList[i].setState(boundaryFollowing);

        Bug newTempBug(boundaryFollowing, lineIds[intersectionPointID].x, lineIds[intersectionPointID].y, bugList[i].getDistance(), bugList[i].getLastPointID(), bugList[i].getBoundaryID());

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
                if(locID == loc.getPrevNode())
                {
                    locationList.updateLocation(locID, loc, true);
                }
                else
                {
                    locationList.updateLocation(locID, loc, false);
                }
                bugList[i].setLastPointID(locID);
            }
        }
        else
        {
            int tempID = locationList.addLocationToList(loc);
            bugList[i].setLastPointID(tempID);
        }


        newTempBug.setBoundaryID(lineId);
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
                if(locID == loc.getPrevNode())
                {
                    locationList.updateLocation(locID, loc, true);
                }
                else
                {
                    locationList.updateLocation(locID, loc, false);
                }
                newTempBug.setLastPointID(locID);
            }
        }
        else
        {
            int tempID = locationList.addLocationToList(loc);
            newTempBug.setLastPointID(tempID);
        }
        bugList.push_back(newTempBug);
    }

    /** adding bug path to goal **/
    else
    {
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
    vector<intersectingPoint> intersectingLinesID;

    intersectingPoint iPoint;

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
    //ROS_INFO("Cost = %f",distance);
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
            calcFinalPath(locationList,finalPath,bug_publisher);
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
    displayBugs(bugList,sourcePoint,bug_publisher);
    bug_publisher.publish(goalPoint);
    bug_publisher.publish(sourcePoint);
    bug_publisher.publish(finalPath);
    displayPaths(pathList, pathMarker, bug_publisher);
    //displayFinalPath(locationList, finalPath, bug_publisher);
    ros::Duration(1).sleep();

    ros::spinOnce();
    return 1;
}
