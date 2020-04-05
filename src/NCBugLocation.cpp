//
// Created by nishant on 13/9/15.
//

#include "path_planning/NCBug.h"

Location::Location() {
  this->prevNode = -1;
  this->currNode = -1;
  this->nextNode = -1;
  this->x = -1;
  this->y = -1;
  this->distance = 0;
}

Location::Location(int prev, int curr, int next, float x, float y,
                   float distance, int bugID) {
  this->prevNode = prev;
  this->currNode = curr;
  this->nextNode = next;
  this->x = x;
  this->y = y;
  this->distance = distance;
  this->addBugtoList(bugID);
}

Location::~Location() {}

void Location::setPrevNode(int pvNode) { this->prevNode = pvNode; }

void Location::setCurrNode(int crNode) { this->currNode = crNode; }

void Location::setNextNode(int nxtNode) { this->nextNode = nxtNode; }

void Location::setBugDetails(vector<int> bugs) { this->bugDetails = bugs; }

void Location::addBugtoList(int bugID) { this->bugDetails.push_back(bugID); }

void Location::setX(float x) { this->x = x; }

void Location::setY(float y) { this->y = y; }

void Location::setDistance(float dist) { this->distance = dist; }

float Location::getDistance() { return this->distance; }

float Location::getX() { return this->x; }

float Location::getY() { return this->y; }

int Location::getPrevNode() { return this->prevNode; }

int Location::getCurrNode() { return this->currNode; }

int Location::getNextNode() { return this->nextNode; }

vector<int> Location::getBugList() { return this->bugDetails; }

void Location::eraseBugDetails() { this->bugDetails.clear(); }

int LocationList::addLocation(Location loc) {
  int id = locationList.size();
  loc.setCurrNode(id);
  loc.setNextNode(-1);
  locationList.push_back(loc);
  locationList[locationList[id].getPrevNode()].setNextNode(id);
  return id;
}

Location LocationList::getLocation(int id) { return this->locationList[id]; }

void LocationList::updateLocation(int id, Location loc, bool sameFlag) {
  // ROS_INFO("updating %d, p %d -- l = %f || c = %f", id, loc.getPrevNode(),
  // locationList[id].getDistance() ,loc.getDistance() );
  int prevID = locationList[id].getPrevNode();

  loc.setCurrNode(id);
  loc.setNextNode(-1);
  locationList[id] = loc;
  if (sameFlag) {
    locationList[id].setPrevNode(prevID);
  }
  locationList[locationList[id].getPrevNode()].setNextNode(id);
}

float LocationList::getDistance(int id) {
  return locationList[id].getDistance();
}

void LocationList::setDistance(int id, float distance) {
  locationList[id].setDistance(distance);
  return;
}

int LocationList::searchLocation(float x, float y) {
  for (int i = 0; i < locationList.size(); i++) {
    if (locationList[i].getX() == x && locationList[i].getY() == y)
      return locationList[i].getCurrNode();
  }
  return -1;
}

void LocationList::eraseBugData(int id) {
  this->locationList[id].eraseBugDetails();
}

void LocationList::removeElement(int id) {
  // unimplemented
}

vector<Location> LocationList::getLocationList() { return this->locationList; }