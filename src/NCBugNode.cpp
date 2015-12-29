//
// Created by nishant on 13/9/15.
//

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <path_planning/NCBug.h>
#include "fstream"

double finalPathDistance = 0;
double preFinalPathDistance = 0 ;

void initializeMarkers(visualization_msgs::Marker &sourcePoint,
                       visualization_msgs::Marker &goalPoint,
                       visualization_msgs::Marker &pathMarker,
                       visualization_msgs::Marker &finalPath)
{
    //init headers
    sourcePoint.header.frame_id    = goalPoint.header.frame_id    = pathMarker.header.frame_id    = finalPath.header.frame_id    = "path_planner";
    sourcePoint.header.stamp       = goalPoint.header.stamp       = pathMarker.header.stamp       = finalPath.header.stamp       = ros::Time::now();
    sourcePoint.ns                 = goalPoint.ns                 = pathMarker.ns                 = finalPath.ns                 = "path_planner_ncbug";
    sourcePoint.action             = goalPoint.action             = pathMarker.action             = finalPath.action             = visualization_msgs::Marker::ADD;
    sourcePoint.pose.orientation.w = goalPoint.pose.orientation.w = pathMarker.pose.orientation.w = finalPath.pose.orientation.w = 1.0;

    //setting id for each marker
    sourcePoint.id    = 0;
    goalPoint.id      = 1;
    pathMarker.id     = 3;
    finalPath.id      = 4;

    //defining types
    pathMarker.type                                       = visualization_msgs::Marker::LINE_LIST;
    finalPath.type                                        = visualization_msgs::Marker::LINE_STRIP;
    sourcePoint.type  = visualization_msgs::Marker::POINTS;
    goalPoint.type = visualization_msgs::Marker::SPHERE;

    //setting scale
    pathMarker.scale.x    = 0.2;
    finalPath.scale.x     = 1;
    sourcePoint.scale.x   = goalPoint.scale.x = 2;
    sourcePoint.scale.y   = goalPoint.scale.y = 2;
    sourcePoint.scale.z   = goalPoint.scale.z = 2;

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

/*
 * Display bugs while execution
 */
void displayBugs(Point source, vector<NCBug> &bugList, visualization_msgs::Marker &sourcePoint, ros::Publisher &bug_publisher)
{
    vector < geometry_msgs::Point> points;
    geometry_msgs::Point point;

    /** adding Source to visual bug list **/
    point.x = source.x;
    point.y = source.y;
    point.z = 0;
    points.push_back(point);

    /** adding bugs to list
    * this loop only matters if function called during run
    */
    for(int i=0;i<bugList.size();i++)
    {
        point.x = bugList[i].getCurPos().x;
        point.y = bugList[i].getCurPos().y;
        point.z = 0;
        points.push_back(point);
    }
    /** publish **/
    sourcePoint.points = points;
    bug_publisher.publish(sourcePoint);
}

/*
 * Function to display paths
 */
void displayPaths(vector< geometry_msgs::Point > &points,visualization_msgs::Marker &pathMarker,ros::Publisher &bug_publisher)
{
    pathMarker.points = points;
    bug_publisher.publish(pathMarker);
}

/*
 * function to generate final path from the dijkstra graph
 */
void calcFinalPath(Point source, LocationList &locationList,visualization_msgs::Marker &finalPath,ros::Publisher &bug_publisher)
{
    /** defining points array for final path **/
    vector< geometry_msgs::Point > points;
    geometry_msgs::Point point;

    /** calculating path by traversing back **/
    int i=1;
    vector<Location> loc = locationList.getLocationList();
    ROS_INFO("Original Path Cost = %f",loc[1].getDistance());
    preFinalPathDistance = loc[1].getDistance();
    while(i != 0)
    {
        point.x = loc[i].getX();
        point.y = loc[i].getY();
        points.push_back(point);
        i = loc[i].getPrevNode();
       // cout<<"i "<< i << endl;
    }
    /** adding Source to final path list **/
    point.x = source.x;
    point.y = source.y;
    points.push_back(point);

   // cout<<"final path lines "<<points.size()<<endl;

    finalPath.points = points;
    bug_publisher.publish(finalPath);
}

/*
 * function to check if there is an intersection between two lines or not
 */
bool getLineIntersections(Point source, Point end, vector < NCOLine > &obstacleLines)
{
    float p0_x = source.x;
    float p0_y = source.y;
    float p1_x = end.x;
    float p1_y = end.y;

    float p2_x;
    float p2_y;
    float p3_x;
    float p3_y;

    float i_x,i_y;
    vector< IntersectionPoint > intersectingLinesID;

    IntersectionPoint iPoint;

    for(int i=0; i< obstacleLines.size();i++)
    {
        p2_x = obstacleLines[i].start.x;
        p2_y = obstacleLines[i].start.y;
        p3_x = obstacleLines[i].end.x;
        p3_y = obstacleLines[i].end.y;

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

            if(i_x == p2_x && i_y == p2_y) continue;
            if(i_x == p3_x && i_y == p3_y) continue;

            //if(i_x == p0_x && i_y == p0_y) continue;
            //if(i_x == p1_x && i_y == p1_y) continue;

            return true;
        }
    }
    return false; // No collision
}

/*
 * function to prun a path
 */
void pruningFinalPath(vector < NCOLine > &obstacleLines, visualization_msgs::Marker &finalPath)
{
	//path is actually reversed and it will not give proper pruning this way so reveresering to tget pruning properly
	reverse(finalPath.points.begin(),finalPath.points.end());

    vector< Point > prunedPath;
    finalPathDistance = 0;
    prunedPath.push_back(finalPath.points[0]);

	cout<< finalPath.points[6].x << " - "<<finalPath.points[6].y<<endl;

    bool intersectionResult = false;

//    for(int i=1;i<finalPath.points.size();i++)
//    {
//        intersectionResult = getLineIntersections(prunedPath[prunedPath.size() - 1], finalPath.points[i],obstacleLines);
//        if(intersectionResult == true) //intersection detected
//        {
//            prunedPath.push_back(finalPath.points[i-1]);
//            finalPathDistance += sqrt(pow(prunedPath[prunedPath.size() - 1].x - prunedPath[prunedPath.size() - 2].x,2)+pow(prunedPath[prunedPath.size() - 1].y - prunedPath[prunedPath.size() - 2].y,2));
//        }
//    }
//    prunedPath.push_back(finalPath.points[finalPath.points.size()-1]);
//    finalPathDistance += sqrt(pow(prunedPath[prunedPath.size() - 1].x - prunedPath[prunedPath.size() - 2].x,2)+pow(prunedPath[prunedPath.size() - 1].y - prunedPath[prunedPath.size() - 2].y,2));

    int i=0;
	int j;
	int lastIntersection = 1;
    while (i < finalPath.points.size()-1)
    {
		lastIntersection = i+1;
        for(j=i+2;j<finalPath.points.size();j++)
        {
            intersectionResult = getLineIntersections(prunedPath[prunedPath.size() - 1], finalPath.points[j],obstacleLines);
            if(intersectionResult == false) //intersection not detected
            {
             	lastIntersection = j;
				cout<<"wow i "<<i<<" wow j "<<j<<endl;
            }
        }
		i = lastIntersection;
		cout<<"i is "<<i<<" given overall size "<<finalPath.points.size()<<endl;
		//if(i>=7) break;
		prunedPath.push_back(finalPath.points[i]);
		finalPathDistance += sqrt(pow(prunedPath[prunedPath.size() - 1].x - prunedPath[prunedPath.size() - 2].x,2)+pow(prunedPath[prunedPath.size() - 1].y - prunedPath[prunedPath.size() - 2].y,2));
	}
	if(i == (finalPath.points.size()-2))
	{
		prunedPath.push_back(finalPath.points[finalPath.points.size() - 1]);
		finalPathDistance += sqrt(pow(prunedPath[prunedPath.size() - 1].x - prunedPath[prunedPath.size() - 2].x, 2) +
								  pow(prunedPath[prunedPath.size() - 1].y - prunedPath[prunedPath.size() - 2].y, 2));
	}

    finalPath.points = prunedPath;
    //ROS_INFO("Cost = %f",distance);
}


int main(int argc, char** argv)
{

    if(argc != 4)// Check the value of passedArgumentCount. if filename is not passed
    {
        std::cout << "usage -> rosrun path_planning nodeName <filename1> <filename2> <filename3>\n"; // Inform the user of how to use the program
        exit(0);
    }

    //initializing ROS
    ros::init(argc,argv,"nc_bug_node");
    ros::NodeHandle n;

    //defining Publisher
    ros::Publisher bug_publisher = n.advertise<visualization_msgs::Marker>("path_planner_bug",1);
    ros::Duration(1).sleep();
    //defining markers
    visualization_msgs::Marker sourcePoint;
    visualization_msgs::Marker goalPoint;
    visualization_msgs::Marker pathMarker;
    visualization_msgs::Marker finalPath;

    Point source;
    Point goal;

    NCOReader obstacle(argv[1],argv[2]);
    vector < NCOLine > obstacleLines = obstacle.getObstacleLines();
    vector< vector< Point > > obstacleList;
    obstacleList = obstacle.getObstacleRects();

    obstacle.setSourceGoalPoints(argv[3],source,goal);

    initializeMarkers(sourcePoint, goalPoint, pathMarker, finalPath);

    goalPoint.pose.position.x = goal.x;
    goalPoint.pose.position.y = goal.y;

    /** initializing bugs**/
    vector< NCBug > bugList;
    vector<NCOLine> tempPath;
    NCBug mainBug(movingToGoal,source,0,0,-1,tempPath);
    bugList.push_back(mainBug);

    LocationList locationList;
    Location loc1(0,0,0,source.x,source.y,0,0);
    locationList.addLocation(loc1);

    Location loc2(0,1,0,goal.x,goal.y,99999,0);
    locationList.addLocation(loc2);

    vector< geometry_msgs::Point > pathList;

    vector<int> idsToRemove;

    ros::Time time = ros::Time::now();

    //algorithm
    while(ros::ok())
    {
        displayBugs(source,bugList,sourcePoint,bug_publisher);
         displayPaths(pathList, pathMarker, bug_publisher);
        ros::Duration(0.025).sleep();
        //remove all the bugs that are on goal or are killed
        //clear the kill list
        removeKilledBugs(bugList,idsToRemove);
        idsToRemove.clear();

        //if all the bugs are killed
        if(bugList.size() == 0)
        {
            calcFinalPath(source, locationList, finalPath, bug_publisher);
            break;
        }

        //call algorithm otherwise for moving bugs in right direction
        for(int i=0;i<bugList.size();i++)
        {

            if(bugList[i].getState() == movingToGoal)
            {
                bool killFlag;
               //cout<<"mg "<<bugList[i].getDistance()<<" "<<i<<endl;
                moveToGoal(bugList, i, goal, obstacleLines, pathList, locationList, killFlag);
                if(killFlag)
                {
                    idsToRemove.push_back(bugList[i].getID());
                }
            }
            else if(bugList[i].getState() == boundaryFollowing)
            {
                bool killFlag;
                //cout<<"bf "<<bugList[i].getDistance()<<" "<<i<<endl;
                boundaryFollow(bugList, i, goal, obstacleLines, obstacleList, pathList,locationList, killFlag);
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
    double prunTime = sec + (nsec / 1000000000.0);
    ROS_INFO("COST %f", finalPathDistance);
    displayBugs(source, bugList,sourcePoint,bug_publisher);
    bug_publisher.publish(goalPoint);
    bug_publisher.publish(sourcePoint);
    displayPaths(pathList, pathMarker, bug_publisher);
    //final path
	bug_publisher.publish(finalPath);
    //calcFinalPath(source, locationList, finalPath, bug_publisher);
    ros::Duration(1).sleep();
    ros::spinOnce();
    ofstream logFile;
    logFile.open("bugLog.txt",ofstream::app);

    logFile << preFinalPathDistance << "," << finalPathDistance <<endl;// << "," << mainTime << "," << prunTime << endl;

    logFile.close();

    vector< geometry_msgs::Point > myPath = finalPath.points;
    //saving path
    logFile.open("bugPath.txt",ofstream::app);

    for(int i=0;i<myPath.size();i++)
    {
        logFile <<myPath[i].x<<","<<myPath[i].y<<endl;
    }

    logFile.close();

    return 1;
}
