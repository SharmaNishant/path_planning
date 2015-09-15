//
// Created by nishant on 13/9/15.
//

#ifndef NISHANT_NCBUG_H
#define NISHANT_NCBUG_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <path_planning/NCOReader.h>
#include <vector>

enum State {movingToGoal, boundaryFollowing, reachedEnd};

using namespace std;

struct IntersectionPoint
{
    int ID;
    float x;
    float y;
};

/*
 * Location in a dijkstra's graph
 */
class Location
{
public:
    Location();
    Location(int prev, int curr, int next, float x, float y, float distance, int bugID);

    ~Location();
    void setPrevNode(int pvNode);
    void setCurrNode(int crNode);
    void setNextNode(int nxtNode);
    void setBugDetails(vector<int> bugs);
    void addBugtoList(int bugID);
    void setX(float x);
    void setY(float y);
    void setDistance(float dist);
    void eraseBugDetails();

    float getDistance();
    float getX();
    float getY();
    int getPrevNode();
    int getCurrNode();
    int getNextNode();
    vector<int> getBugList();

protected:
private:
    int prevNode;
    int currNode;
    int nextNode;
    float x;
    float y;
    float distance;
    vector<int> bugDetails;
};


/*
 * Dijkstra's graph as a list
 */
class LocationList
{
public:
    LocationList() {};
    ~LocationList() {};

    int addLocation(Location loc);

    Location getLocation(int id);

    void updateLocation(int id, Location loc, bool sameFlag);

    float getDistance(int id);
    void setDistance(int id, float distance);

    int searchLocation(float x, float y);

    void eraseBugData(int id);
    void removeElement(int id); //unimplemented

    vector<Location> getLocationList();

    private:
        vector<Location> locationList;
};

/*
 * Bug details
 */
class NCBug
{
public:
    NCBug(State state, Point loc,float dist, int lastPTID, int boundID, vector<NCOLine> path);
    ~NCBug();

    //set functions
    void setState(State state);

    void setCurPos(Point pos);

    void setLastPos(Point pos);
    void setTempGoal(Point pos);

    void setLastPointID(int Id);
    void setBoundaryID(int boundaryId);

    void setDistance(float dist);
    void addDistance(float dist);

    void setPath(vector< NCOLine > path);
    void addPath(NCOLine line);

    //getFunctions
    State getState();

    int getID();

    Point getCurPos();
    Point getLastPos();
    Point getTempGoal();

    int getBoundaryID();
    float getDistance();
    int getLastPointID();

    vector< NCOLine > getPath();

    //check if bug on line of the obstacle
    int checkIfOnOtherLines(vector < NCOLine > &obstacleLines);

    //check if the new line is inside
    bool checkInsideObstacle(vector< vector< Point> > &obstArray, Point goal);

    vector<IntersectionPoint> getLineIntersections(Point goal, vector < NCOLine > &obstacleLines);

    float getEuclideanDistance(Point p0, Point p1);

protected:
private:

    //for assigning ID to bug at construction
    static int bugIDCounter;

    //bug member variables

    //bug ID
    int ID;

    //current pos of bug
    Point curPos;

    //last position of bug
    Point lastPos;

    //for boundary following
    Point tempGoal;

    //last point where bug was
    int lastPointID;

    //current boundary line on which bug is following the line
    int boundaryID;

    //current distance travelled by the bug
    float distance;

    //current state of the bug
    State state;

    //to store path covered by the bug till now
    vector< NCOLine > visitedPath;
};

#define bugStepSize 1

void removeKilledBugs(vector< NCBug > &bugList, vector<int> idsToRemove);

void moveToGoal(vector<NCBug> &bugList, int i, Point goal, vector < NCOLine > &obstacleLines,
                vector< geometry_msgs::Point > &pathList, LocationList &locationList, bool &killFlag);

void boundaryFollow(vector<NCBug> &bugList,int i, Point goal, vector < NCOLine > &obstacleLines,  vector< vector<geometry_msgs::Point> > &obstArray,
                    vector< geometry_msgs::Point > &pathList, LocationList &locationList, bool &killFlag);

#endif //NISHANT_NCBUG_H
