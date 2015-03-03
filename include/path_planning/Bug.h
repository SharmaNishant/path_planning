#ifndef BUG_H
#define BUG_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <path_planning/obstacles.h>
#include <vector>

enum State {movingToGoal, boundaryFollowing, reachedEnd};

using namespace std;

struct line{

    float pointTwoX;
    float pointTwoy;
    float pointOneX;
    float pointOneY;

};

struct intersectingPoint
{
    int ID;
    float x;
    float y;
};

class location
{
    public:
        location();
        location(int prev, int curr, int next, float x, float y, float distance, int bugID);

        ~location();
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

class LocationArray
{
    public:
        int addLocationToList(location loc);
        location getElement(int id) { return locationList[id];};
        void updateLocation(int id, location loc, bool sameFlag);
        float getDistance(int id);
        void setDistance(int id, float distance);
        int searchLocation(float x, float y);
        void eraseBugData(int id);
        void removeElement(int id);
        vector<location> getLocationList() { return locationList;};
    protected:
    private:
        vector<location> locationList;
};

class Bug
{
    public:
        Bug(State state, float x, float y,float dist, int lastPTID, int boundID);
        ~Bug();

        void setState(State state);
        void setXY(float x, float y);
        void setX(float x);
        void setY(float y);
        void setLastX(float x);
        void setLastY(float y);
        void setLastPointID(int Id);
        void setBoundaryID(int boundaryId);
        void setDistance(float dist);
        void addDistance(float dist);
        void setNext(Bug *next);
        void setPrev(Bug *prev);

        State getState();
        int getID();
        float getX();
        float getY();
        float getLastX();
        float getLastY();
        int getBoundaryID();
        float getDistance();
        int getLastPointID();

        /** Actual Bug Function */
       //void moveBug(float goalX, float goalY, vector < obstacleLine > &obstacleLines, vector< vector<geometry_msgs::Point> > &obstacleList,
       //             int &pointID, int &pointIDOne,int &pointIDTwo, LocationArray &locationList, bool &killFlag, Bug** newbug);


        int checkIfOnOtherLines(vector < obstacleLine > &obstacleLines, int &index);
        bool checkInsideObstacles(vector< vector<geometry_msgs::Point> > &obstArray, float goalX, float goalY);
        vector<intersectingPoint> getLineIntersections(float goalX, float goalY, vector < obstacleLine > &obstacleLines);
        float getEuclideanDistance(float p0_x, float p0_y, float p1_x, float p1_y);

    protected:
    private:

        static int bugIDCounter;

        int ID;
        float x;
        float y;
        float lastX;
        float lastY;
        int lastPointID;
        int boundaryID;
        float distance;
        State state;

};
#endif // BUG_H
