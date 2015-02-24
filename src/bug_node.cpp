#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <path_planning/bug.h>
#include <path_planning/obstacles.h>
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

void displayBugs(vector<bug> bugs, visualization_msgs::Marker &sourcePoint, ros::Publisher &bug_publisher)
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
       // points.push_back(point);
    }
    sourcePoint.points = points;
    bug_publisher.publish(sourcePoint);
}

// return minimum distance between line segment vw and point p, and the closest point on the line segment, q
float DistanceFromLineSegmentToPoint( const Vec2 v, const Vec2 w, const Vec2 p, Vec2 * const q )
{
    const float distSq = v.DistanceToSquared( w ); // i.e. |w-v|^2 ... avoid a sqrt
    if ( distSq == 0.0 )
    {
        // v == w case
        (*q) = v;

        return v.DistanceTo( p );
    }

    // consider the line extending the segment, parameterized as v + t (w - v)
    // we find projection of point p onto the line
    // it falls where t = [(p-v) . (w-v)] / |w-v|^2

    const float t = ( p - v ).DotProduct( w - v ) / distSq;
    if ( t < 0.0 )
    {
        // beyond the v end of the segment
        (*q) = v;

        return v.DistanceTo( p );
    }
    else if ( t > 1.0 )
    {
        // beyond the w end of the segment
        (*q) = w;

        return w.DistanceTo( p );
    }

    // projection falls on the segment
    const Vec2 projection = v + ( ( w - v ) * t );

    (*q) = projection;

    return p.DistanceTo( projection );
}

float DistanceFromLineSegmentToPoint( float segmentX1, float segmentY1, float segmentX2, float segmentY2, float pX, float pY, float *qX, float *qY )
{
    Vec2 q;

    float distance = DistanceFromLineSegmentToPoint( Vec2( segmentX1, segmentY1 ), Vec2( segmentX2, segmentY2 ), Vec2( pX, pY ), &q );

    (*qX) = q._x;
    (*qY) = q._y;

    return distance;
}

bool checkInsideObstacles(vector< vector<geometry_msgs::Point> > &obstArray, double newX, double newY)
{
    double AB, AD, AMAB, AMAD;

    for(int i=0; i<obstArray.size(); i++)
    {
        AB = (pow(obstArray[i][0].x - obstArray[i][1].x,2) + pow(obstArray[i][0].y - obstArray[i][1].y,2));
        AD = (pow(obstArray[i][0].x - obstArray[i][3].x,2) + pow(obstArray[i][0].y - obstArray[i][3].y,2));
        AMAB = (((newX - obstArray[i][0].x) * (obstArray[i][1].x - obstArray[i][0].x)) + ((newY - obstArray[i][0].y) * (obstArray[i][1].y - obstArray[i][0].y)));
        AMAD = (((newX - obstArray[i][0].x) * (obstArray[i][3].x - obstArray[i][0].x)) + ((newY - obstArray[i][0].y) * (obstArray[i][3].y - obstArray[i][0].y)));
         //(0<AM⋅AB<AB⋅AB)∧(0<AM⋅AD<AD⋅AD)
        if((0 < AMAB) && (AMAB < AB) && (0 < AMAD) && (AMAD < AD))
        {
            return true;
        }
    }
    return false;
}

float checkDistancetoObstacleBoundary(vector < obstacleLine > obstacleLines, float newX, float newY, float &pointX, float &pointY, vector<int> &lineId)
{
    double mini = 2,result;
    for(int i=0; i< obstacleLines.size(); i++)
    {
        result = DistanceFromLineSegmentToPoint(obstacleLines[i].point[0].x,obstacleLines[i].point[0].y,obstacleLines[i].point[1].x,obstacleLines[i].point[1].y,newX,newY,&pointX,&pointY);
        if(result < mini)
        {
            //mini = result;
            lineId.push_back(i);
        }
    }
    //ROS_INFO("in function %f, %d", mini, lineId);
    return mini;
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


bool isNearGoal(double x, double y)
{
    double dist = sqrt(pow(goalX - x,2)+pow(goalY-y,2));

    if(dist < 2)
        return true;
    else
        return false;
}

bool get_line_intersection(float p0_x, float p0_y, float p1_x, float p1_y,
    float p2_x, float p2_y, float p3_x, float p3_y)
{
    float s02_x, s02_y, s10_x, s10_y, s32_x, s32_y, s_numer, t_numer, denom, t;
    s10_x = p1_x - p0_x;
    s10_y = p1_y - p0_y;
    s02_x = p0_x - p2_x;
    s02_y = p0_y - p2_y;

    s_numer = s10_x * s02_y - s10_y * s02_x;
    if (s_numer < 0)
        return false; // No collision

    s32_x = p3_x - p2_x;
    s32_y = p3_y - p2_y;
    t_numer = s32_x * s02_y - s32_y * s02_x;
    if (t_numer < 0)
        return false; // No collision

    denom = s10_x * s32_y - s32_x * s10_y;
    if (s_numer > denom || t_numer > denom)
        return false; // No collision

    return true;
}

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
}

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

        if(path_length < min_path)
        {
            min_path = path_length;
            minPathId = i;
        }

    }


    path = bugs[minPathId].getPath();
    for(int j=1; j<bugs[minPathId].getPathLength();j++)
    {
        point.x = path[j-1].x;
        point.y = path[j-1].y;
        points.push_back(point);
        point.x = path[j].x;
        point.y = path[j].y;
        points.push_back(point);
    }
    finalPath.points = points;
    bug_publisher.publish(finalPath);
}


int main(int argc, char** argv)
{
    //initializing ROS
    ros::init(argc,argv,"rrt_node");
	ros::NodeHandle n;

	//defining Publisher
	ros::Publisher bug_publisher = n.advertise<visualization_msgs::Marker>("path_planner_rrt",1);

	//defining markers
    visualization_msgs::Marker sourcePoint;
    visualization_msgs::Marker goalPoint;
    visualization_msgs::Marker pathMarker;
    visualization_msgs::Marker finalPath;

    initializeMarkers(sourcePoint, goalPoint, pathMarker, finalPath);

    goalPoint.pose.position.x = 95;
    goalPoint.pose.position.y = 95;

    vector<bug> bugs;
    bug mainBug;

    mainBug.setPosX(sourceX);
    mainBug.setPosY(sourceY);
    mainBug.setLastX(sourceX);
    mainBug.setLastY(sourceY);
    location loc;
    loc.x = sourceX;
    loc.y = sourceY;
    mainBug.addNodeToPath(loc);
    mainBug.setState(movingToGoal);
    bugs.push_back(mainBug);

    obstacles obstacle;
    vector < obstacleLine > obstacleLines = obstacle.getObstacleLines();
    vector< vector<geometry_msgs::Point> > obstacleList = obstacle.getObstacleArray();

    int bugsCompleteCounter = 0;

    while(ros::ok())
    {
        displayBugs(bugs, sourcePoint,bug_publisher);
        bugsCompleteCounter = 0;
        for(int i=0; i < bugs.size(); i++)
        {
            if(isNearGoal(bugs[i].getPosX(),bugs[i].getPosY()))
            {
                loc.x = goalX;
                loc.y = goalY;
                bugs[i].addNodeToPath(loc);

                bugsCompleteCounter++;
                bugs[i].setState(reachedEnd);
            }

            if (bugs[i].getState() == movingToGoal)
            {

                //ROS_INFO("robot position , %d - %f || %f ", i,bugs[i].getPosX() , bugs[i].getPosY() );

                double theta = atan2(goalY - bugs[i].getPosY(), goalX - bugs[i].getPosX());
                float newX = bugs[i].getPosX() + ( bugs[i].getStepSize() * cos(theta));
                float newY = bugs[i].getPosY() + ( bugs[i].getStepSize() * sin(theta));

                //ROS_INFO("theta, %f - %f || %f ", newX,newY, theta);

                float pointX,pointY;
                vector<int> lineIds;
                float distance = checkDistancetoObstacleBoundary(obstacleLines, newX, newY,pointX,pointY, lineIds);

                 //ROS_INFO("line, %f - %f || %d || %f", pointX,pointX, lineId, distance );
                int lineId;
                bool lineFlag = false;
                for(int j=0;j< lineIds.size();j++)
                {
                    lineId = lineIds[j];
                    if(get_line_intersection(obstacleLines[lineId].point[0].x, obstacleLines[lineId].point[0].y,
                            obstacleLines[lineId].point[1].x, obstacleLines[lineId].point[1].y,
                            goalX, goalY, bugs[i].getPosX(), bugs[i].getPosY()))
                    {
                        lineFlag = true;
                        break;
                    }
                }
                if(lineFlag)
                {
//                    ROS_INFO("in move goal if d <2, %f - %f || %f ", newX,newY, distance );

                    //point on the line
                    loc.x = pointX;
                    loc.y = pointY;

                    bugs[i].addNodeToPath(loc);
                    bugs[i].setBoundaryID(lineId);
                    bugs[i].setState(boundaryFollowing);

                    bug newTempBug(boundaryFollowing, pointX, pointY, bugs[i].getPath());
                    newTempBug.setLastX(pointX);
                    newTempBug.setLastY(pointY);
                    newTempBug.setBoundaryID(lineId);

                    newTempBug.setPosX(obstacleLines[lineId].point[0].x);
                    newTempBug.setPosY(obstacleLines[lineId].point[0].y);
                    loc.x = newTempBug.getPosX();
                    loc.y = newTempBug.getPosY();
                    newTempBug.addNodeToPath(loc);
                    bugs.push_back(newTempBug);


                    bugs[i].setPosX(obstacleLines[lineId].point[1].x);
                    bugs[i].setPosY(obstacleLines[lineId].point[1].y);
                    loc.x = bugs[i].getPosX();
                    loc.y = bugs[i].getPosY();
                    bugs[i].addNodeToPath(loc);

                }
                else
                {
//                    ROS_INFO("in move goal else");
                    bugs[i].setPosX(newX);
                    bugs[i].setPosY(newY);
                }
            }
            if(bugs[i].getState() == boundaryFollowing)
            {
                double theta = atan2(goalY - bugs[i].getPosY(), goalX - bugs[i].getPosX());
                float newX = bugs[i].getPosX() + ( bugs[i].getStepSize() * cos(theta));
                float newY = bugs[i].getPosY() + ( bugs[i].getStepSize() * sin(theta));

                if( checkInsideObstacles(obstacleList, newX, newY))
                {
                    int index;
                    double newerX = bugs[i].getPosX();
                    double newerY = bugs[i].getPosY();
                    int line = checkOtherLines(obstacleLines, newerX, newerY,index, bugs[i].getBoundaryID());
                    if(index == 5) exit(1);
                    bugs[i].setPosX(obstacleLines[line].point[index].x);
                    bugs[i].setPosY(obstacleLines[line].point[index].y);
                    loc.x = bugs[i].getPosX();
                    loc.y = bugs[i].getPosY();
                    bugs[i].addNodeToPath(loc);
                }
                else
                {
                    bugs[i].setState(movingToGoal);
                    bugs[i].setMoveToGoalFlag(true);
                    bugs[i].setPosX(newX);
                    bugs[i].setPosY(newY);
                    bugs[i].setLastX(bugs[i].getPosX());
                    bugs[i].setLastY(bugs[i].getPosY());
                }
            }
        }

        if(bugsCompleteCounter == bugs.size())
        {
            break;
        }
    bug_publisher.publish(goalPoint);
    //ros::Duration(0.1).sleep();
    }
    bug_publisher.publish(goalPoint);
    displayPaths(bugs, pathMarker, bug_publisher);
    displayFinalPath(bugs, finalPath, bug_publisher);
    ros::Duration(0.1).sleep();
//    displayPaths();


    return 1;
}
