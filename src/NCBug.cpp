//
// Created by nishant on 13/9/15.
//

#include "path_planning/NCBug.h"

int NCBug::bugIDCounter = 0;

NCBug::NCBug(State state, Point loc, float dist, int lastPTID, int boundID,
             vector<NCOLine> path) {
  this->state = state;
  this->curPos.x = loc.x;
  this->curPos.y = loc.y;
  this->ID = NCBug::bugIDCounter++;
  this->lastPos.x = loc.x;
  this->lastPos.y = loc.y;
  this->lastPointID = lastPTID;
  this->boundaryID = boundID;
  this->distance = dist;
  this->visitedPath = path;
}

NCBug::~NCBug() {
  // dtor
}

/**
    Set Functions
*/

void NCBug::setState(State state) { this->state = state; }

/*
 * sets current position and add it to the visited path list
 */
void NCBug::setCurPos(Point pos) {
  NCOLine ncoLine;
  ncoLine.start = this->getCurPos();
  ncoLine.end = pos;
  this->addPath(ncoLine);
  this->setLastPos(this->getCurPos());
  this->curPos = pos;
}

void NCBug::setLastPos(Point pos) { this->lastPos = pos; }

void NCBug::setTempGoal(Point pos) { this->tempGoal = pos; }

void NCBug::setLastPointID(int Id) { this->lastPointID = Id; }

void NCBug::setBoundaryID(int boundaryId) { this->boundaryID = boundaryId; }

void NCBug::setDistance(float dist) { this->distance = dist; }

void NCBug::addDistance(float dist) { this->distance += dist; }

void NCBug::setPath(vector<NCOLine> path) { this->visitedPath = path; }

void NCBug::addPath(NCOLine line) { this->visitedPath.push_back(line); }

/**
    Get Functions
*/

State NCBug::getState() { return this->state; }

int NCBug::getID() { return this->ID; }

Point NCBug::getCurPos() { return this->curPos; }

Point NCBug::getLastPos() { return this->lastPos; }

Point NCBug::getTempGoal() { return this->tempGoal; }

int NCBug::getBoundaryID() { return this->boundaryID; }

float NCBug::getDistance() { return this->distance; }

int NCBug::getLastPointID() { return this->lastPointID; }

vector<NCOLine> NCBug::getPath() { return this->visitedPath; }

/**
    Bug Functions
*/

bool NCBug::checkInsideObstacle(vector<vector<Point>> &obstArray, Point goal) {
  double theta =
      atan2(goal.y - this->getCurPos().y, goal.x - this->getCurPos().x);
  double newX = this->getCurPos().x + (1 * cos(theta));
  double newY = this->getCurPos().y + (1 * sin(theta));
  double AB, AD, AMAB, AMAD;

  // cout<<"cur point "<< newX << " "<< newY<<endl;

  for (int i = 0; i < obstArray.size(); i++) {
    AB = (pow(obstArray[i][0].x - obstArray[i][1].x, 2) +
          pow(obstArray[i][0].y - obstArray[i][1].y, 2));
    AD = (pow(obstArray[i][0].x - obstArray[i][3].x, 2) +
          pow(obstArray[i][0].y - obstArray[i][3].y, 2));
    AMAB = (((newX - obstArray[i][0].x) *
             (obstArray[i][1].x - obstArray[i][0].x)) +
            ((newY - obstArray[i][0].y) *
             (obstArray[i][1].y - obstArray[i][0].y)));
    AMAD = (((newX - obstArray[i][0].x) *
             (obstArray[i][3].x - obstArray[i][0].x)) +
            ((newY - obstArray[i][0].y) *
             (obstArray[i][3].y - obstArray[i][0].y)));
    //(0<AM⋅AB<AB⋅AB)∧(0<AM⋅AD<AD⋅AD)
    if ((0 < AMAB) && (AMAB < AB) && (0 < AMAD) && (AMAD < AD)) {
      // cout<<"returing true\n";
      return true;
    }
  }
  // cout<<"returing false\n";
  return false;
}

int NCBug::checkIfOnOtherLines(vector<NCOLine> &obstacleLines) {
  for (int i = 0; i < obstacleLines.size(); i++) {
    if (i == this->boundaryID) {
      continue;
    }
    if (obstacleLines[i].start.x == this->getCurPos().x &&
        obstacleLines[i].start.y == this->getCurPos().y) {
      // set which direction it should move in now
      this->setTempGoal(obstacleLines[i].end);
      this->setBoundaryID(i);
      return i;
    }
    if (obstacleLines[i].end.x == this->getCurPos().x &&
        obstacleLines[i].end.y == this->getCurPos().y) {
      // set which direction it should move in now
      this->setTempGoal(obstacleLines[i].start);
      this->setBoundaryID(i);
      return i;
    }
  }
  return -1;
}

vector<IntersectionPoint>
NCBug::getLineIntersections(Point goal, vector<NCOLine> &obstacleLines) {
  float p0_x = this->getCurPos().x;
  float p0_y = this->getCurPos().y;
  float p1_x = goal.x;
  float p1_y = goal.y;

  float p2_x;
  float p2_y;
  float p3_x;
  float p3_y;

  float i_x, i_y;
  vector<IntersectionPoint> intersectingLinesID;

  IntersectionPoint iPoint;

  for (int i = 0; i < obstacleLines.size(); i++) {
    p2_x = obstacleLines[i].start.x;
    p2_y = obstacleLines[i].start.y;
    p3_x = obstacleLines[i].end.x;
    p3_y = obstacleLines[i].end.y;
    float s1_x, s1_y, s2_x, s2_y;
    s1_x = p1_x - p0_x;
    s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;
    s2_y = p3_y - p2_y;

    float s, t;
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) /
        (-s2_x * s1_y + s1_x * s2_y);
    t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) /
        (-s2_x * s1_y + s1_x * s2_y);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
      // Collision detected
      i_x = p0_x + (t * s1_x);
      i_y = p0_y + (t * s1_y);

      if (i_x == p0_x && i_y == p0_y)
        continue;

      iPoint.ID = i;
      iPoint.x = i_x;
      iPoint.y = i_y;
      intersectingLinesID.push_back(iPoint);
    }
  }
  return intersectingLinesID; // No collision
}

float NCBug::getEuclideanDistance(Point p0, Point p1) {
  return sqrt(pow(p1.x - p0.x, 2) + pow(p1.y - p0.y, 2));
}
