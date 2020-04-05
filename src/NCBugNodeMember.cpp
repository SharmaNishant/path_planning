//
// Created by nishant on 13/9/15.
//

#include "path_planning/NCBug.h"

void removeKilledBugs(vector<NCBug> &bugList, vector<int> idsToRemove) {
  for (int i = 0; i < bugList.size();) {

    bool flaged = false;
    for (int j = 0; j < idsToRemove.size(); j++) {
      if (idsToRemove[j] == bugList[i].getID()) {
        bugList.erase(bugList.begin() + i);
        flaged = true;
        break;
      }
    }
    if (flaged)
      continue;
    if (bugList[i].getState() == reachedEnd) {
      bugList.erase(bugList.begin() + i);
      continue;
    }
    i++;
  }
}

void moveToGoal(vector<NCBug> &bugList, int i, Point goal,
                vector<NCOLine> &obstacleLines,
                vector<geometry_msgs::Point> &pathList,
                LocationList &locationList, bool &killFlag) {
  geometry_msgs::Point point;
  killFlag = false;
  Location loc;
  vector<IntersectionPoint> lineIds;
  lineIds = bugList[i].getLineIntersections(goal, obstacleLines);

  /** routine for getting nearest line **/
  int lineId;
  int intersectionPointID;
  bool lineFlag = false;
  float minDist = 9999;
  float dist;
  for (int j = 0; j < lineIds.size(); j++) {
    Point end;
    end.x = lineIds[j].x;
    end.y = lineIds[j].y;
    dist = bugList[i].getEuclideanDistance(bugList[i].getCurPos(), end);

    if (dist < minDist) {
      minDist = dist;
      lineId = lineIds[j].ID;
      intersectionPointID = j;
      lineFlag = true;
    }
  }

  // cout<<"bug "<<i<<" dist from line "<<minDist<<endl;

  // there is a line ahead but we have free space for one step
  if (lineFlag && minDist > bugStepSize) {
    Point nextPoint;
    double angleGoal = atan2(goal.y - bugList[i].getCurPos().y,
                             goal.x - bugList[i].getCurPos().x);
    nextPoint.x = bugList[i].getCurPos().x + (bugStepSize * cos(angleGoal));
    nextPoint.y = bugList[i].getCurPos().y + (bugStepSize * sin(angleGoal));

    // can add a check if inside polynomial but it will be mostly redundant
    bugList[i].setCurPos(nextPoint);
    bugList[i].addDistance(bugStepSize);

    // adding it to visible path
    pathList.push_back(bugList[i].getCurPos());
    pathList.push_back(bugList[i].getLastPos());

    // std::cout<<"Moving one step - "<< i << std::endl;
  }
  /** if there is a line in the way **/
  if (lineFlag && minDist <= bugStepSize) {
    // cout<<"Entering intersection "<<i<<endl;
    Point newPos;
    newPos.x = lineIds[intersectionPointID].x;
    newPos.y = lineIds[intersectionPointID].y;
    bugList[i].setCurPos(newPos);

    bugList[i].addDistance(minDist);
    // cout<<minDist<<" "<<i<<"\n";
    // exit(1);
    // adding it to visible path
    pathList.push_back(bugList[i].getCurPos());
    pathList.push_back(bugList[i].getLastPos());

    Location loc;
    loc.setX(lineIds[intersectionPointID].x);
    loc.setY(lineIds[intersectionPointID].y);
    loc.setDistance(bugList[i].getDistance());
    loc.setPrevNode(bugList[i].getLastPointID());
    loc.addBugtoList(bugList[i].getID());

    /*** adding intersection point ***/
    int locID = locationList.searchLocation(lineIds[intersectionPointID].x,
                                            lineIds[intersectionPointID].y);
    // cout<<"location id "<<locID<<endl;
    // ros::Duration(1).sleep();
    if (locID != -1) {
      // cout<<"location updated"<<endl;
      float prevDistanceToPoint = locationList.getDistance(locID);
      if (prevDistanceToPoint <= loc.getDistance() &&
          prevDistanceToPoint != -1) {
        killFlag = true;
        return;
      } else {
        if (locID == loc.getPrevNode()) {
          locationList.updateLocation(locID, loc, true);
        } else {
          locationList.updateLocation(locID, loc, false);
        }
        bugList[i].setLastPointID(locID);
      }
    } else {
      // cout<<loc.getPrevNode()<<" i "<<endl;
      int tempID = locationList.addLocation(loc);
      bugList[i].setLastPointID(tempID);
      // cout<<"location updated : total
      // "<<locationList.getLocationList().size()<<endl; cout<<"temp id " <<
      // tempID << endl;
    }

    /** changing state **/
    bugList[i].setBoundaryID(lineId);
    bugList[i].setState(boundaryFollowing);
    bugList[i].setTempGoal(obstacleLines[lineId].start);

    /* for new created bug */
    newPos.x = lineIds[intersectionPointID].x;
    newPos.y = lineIds[intersectionPointID].y;
    NCBug newTempBug(boundaryFollowing, newPos, bugList[i].getDistance(),
                     bugList[i].getLastPointID(), bugList[i].getBoundaryID(),
                     bugList[i].getPath());
    newTempBug.setBoundaryID(lineId);
    newTempBug.setTempGoal(obstacleLines[lineId].end);
    bugList.push_back(newTempBug);
  }

  /** if no intersection then add bug path to goal **/
  if (!lineFlag) {
    bugList[i].setState(reachedEnd);
    killFlag = true;

    float distanceTemp =
        bugList[i].getEuclideanDistance(bugList[i].getCurPos(), goal);
    bugList[i].setCurPos(goal);
    bugList[i].addDistance(distanceTemp);

    // adding positions to the path to the path
    pathList.push_back(bugList[i].getCurPos());
    pathList.push_back(bugList[i].getLastPos());

    /** add to path GRAPH **/
    Location loc;
    loc.setX(bugList[i].getCurPos().x);
    loc.setY(bugList[i].getCurPos().y);
    loc.setDistance(bugList[i].getDistance());
    loc.setPrevNode(bugList[i].getLastPointID());
    loc.addBugtoList(bugList[i].getID());

    float prevDistanceToPoint = locationList.getDistance(1);

    if (prevDistanceToPoint > loc.getDistance()) {
      locationList.updateLocation(1, loc, false);
      bugList[i].setLastPointID(1);
    }
    killFlag = true;
  }
}

void boundaryFollow(vector<NCBug> &bugList, int i, Point goal,
                    vector<NCOLine> &obstacleLines,
                    vector<vector<Point>> &obstArray,
                    vector<geometry_msgs::Point> &pathList,
                    LocationList &locationList, bool &killFlag)

{
  /** defining values **/
  geometry_msgs::Point point;
  killFlag = false;
  Location loc;

  float distanceTempGoal = bugList[i].getEuclideanDistance(
      bugList[i].getCurPos(), bugList[i].getTempGoal());

  // cout<<i<<" dist to goal "<<distanceTempGoal<<endl;

  // when you are still following line and the line end is far
  if (distanceTempGoal > bugStepSize) {
    Point nextPoint;
    double angleGoal =
        atan2(bugList[i].getTempGoal().y - bugList[i].getCurPos().y,
              bugList[i].getTempGoal().x - bugList[i].getCurPos().x);
    nextPoint.x = bugList[i].getCurPos().x + (bugStepSize * cos(angleGoal));
    nextPoint.y = bugList[i].getCurPos().y + (bugStepSize * sin(angleGoal));

    // check if inside the obstacle (we are not doing it because we are on a
    // line)

    // can add a check if inside polynomial but it will be mostly redundant
    bugList[i].setCurPos(nextPoint);
    bugList[i].addDistance(bugStepSize);

    // adding positions to the path to the visible path in rviz
    pathList.push_back(bugList[i].getCurPos());
    pathList.push_back(bugList[i].getLastPos());

    // check if you can move towards goal
    vector<IntersectionPoint> lineIds;
    vector<NCOLine> pathLines = bugList[i].getPath();
    lineIds = bugList[i].getLineIntersections(goal, pathLines);

    // cout<<"path intersection "<< lineIds.size() <<endl;
    // if the path is not intersecting with the previous path
    if (lineIds.size() == 0) {
      if (!bugList[i].checkInsideObstacle(obstArray, goal)) {
        // cout<<"not inside obstacle "<<i<<endl;

        Location loc;
        loc.setX(nextPoint.x);
        loc.setY(nextPoint.y);
        loc.setDistance(bugList[i].getDistance());
        loc.setPrevNode(bugList[i].getLastPointID());
        loc.addBugtoList(bugList[i].getID());

        // add this point in the graph
        int locID = locationList.searchLocation(nextPoint.x, nextPoint.y);
        if (locID != -1) {
          // cout<<"location updated"<<endl;
          float prevDistanceToPoint = locationList.getDistance(locID);
          if (prevDistanceToPoint <= loc.getDistance() &&
              prevDistanceToPoint != -1) {
            killFlag = true;
          } else {
            if (locID == loc.getPrevNode()) {
              locationList.updateLocation(locID, loc, true);
            } else {
              locationList.updateLocation(locID, loc, false);
            }
            bugList[i].setLastPointID(locID);
          }
        } else {
          int tempID = locationList.addLocation(loc);
          bugList[i].setLastPointID(tempID);
          // cout<<"location added leaving for goal : total
          // "<<locationList.getLocationList().size()<<endl;
          // ros::Duration(1).sleep();
          // cout<<"temp id " << tempID << endl;
        }
        bugList[i].setState(movingToGoal);
        // cout<<"change state to mg "<< i<< endl;
      }
    }
    // return;
  } else // distanceTempGoal < bugStepSize
  {
    // set bug's current location to temp goal

    bugList[i].setCurPos(bugList[i].getTempGoal());
    bugList[i].addDistance(distanceTempGoal);

    // adding positions to the path to the visible path in rviz
    pathList.push_back(bugList[i].getCurPos());
    pathList.push_back(bugList[i].getLastPos());

    Location loc;
    loc.setX(bugList[i].getCurPos().x);
    loc.setY(bugList[i].getCurPos().y);
    loc.setDistance(bugList[i].getDistance());
    loc.setPrevNode(bugList[i].getLastPointID());
    loc.addBugtoList(bugList[i].getID());

    // add this point in the graph
    int locID = locationList.searchLocation(bugList[i].getCurPos().x,
                                            bugList[i].getCurPos().y);
    if (locID != -1) {
      // cout<<"location added"<<endl;
      float prevDistanceToPoint = locationList.getDistance(locID);
      if (prevDistanceToPoint <= loc.getDistance() &&
          prevDistanceToPoint != -1) {
        killFlag = true;
      } else {
        if (locID == loc.getPrevNode()) {
          locationList.updateLocation(locID, loc, true);
        } else {
          locationList.updateLocation(locID, loc, false);
        }
        bugList[i].setLastPointID(locID);
      }
    } else {
      // cout<<loc.getPrevNode()<<" i "<<i<<endl;
      int tempID = locationList.addLocation(loc);
      bugList[i].setLastPointID(tempID);
      // cout<<"location added line end : total
      // "<<locationList.getLocationList().size()<<endl;
      // ros::Duration(1).sleep();
    }

    /*
     * now that the point is added in the graph
     */

    /** if the bug is clear to move towards goal or not **/
    /** bug is not clear to move **/
    vector<IntersectionPoint> lineIds;
    vector<NCOLine> pathLines = bugList[i].getPath();
    lineIds = bugList[i].getLineIntersections(goal, pathLines);
    if (bugList[i].checkInsideObstacle(obstArray, goal) ||
        lineIds.size() != 0) {
      // cout<<bugList[i].getDistance()<<" "<<i<<endl;
      /** check which obstacle line can be followed **/
      bugList[i].checkIfOnOtherLines(obstacleLines);
    } else {
      bugList[i].setState(movingToGoal);
    }
  }
  // cout<<"exiting bf "<<i<<endl;
}
