#include "path_planning/Bug.h"


location::location()
{
    prevNode = -1;
    currNode = -1;
    nextNode = -1;
    x = -1;
    y = -1;
    distance = 0;
}

location::location(int prev, int curr, int next, float x, float y, float distance, int bugID)
{
    this->prevNode = prev;
    this->currNode = curr;
    this->nextNode = next;
    this->x = x;
    this->y = y;
    this->distance = distance;
    this->addBugtoList(bugID);
}

location::~location()
{

}

void location::setPrevNode(int pvNode)
{
    this->prevNode = pvNode;
}

void location::setCurrNode(int crNode)
{
    this->currNode = crNode;
}

void location::setNextNode(int nxtNode)
{
    this->nextNode = nxtNode;
}

void location::setBugDetails(vector<int> bugs)
{
    this->bugDetails = bugs;
}

void location::addBugtoList(int bugID)
{
    this->bugDetails.push_back(bugID);
}

void location::setX(float x)
{
    this->x = x;
}

void location::setY(float y)
{
    this->y = y;
}

void location::setDistance(float dist)
{
    this->distance = dist;
}

float location::getDistance()
{
    return this->distance;
}

float location::getX()
{
    return this->x;
}

float location::getY()
{
    return this->y;
}

int location::getPrevNode()
{
    return this->prevNode;
}

int location::getCurrNode()
{
    return this->currNode;
}

int location::getNextNode()
{
    return this->nextNode;
}

vector<int> location::getBugList()
{
    return this->bugDetails;
}

void location::eraseBugDetails()
{
    this->bugDetails.clear();
}


int LocationArray::addLocationToList(location loc)
{
    int id = locationList.size();
    loc.setCurrNode(id);
    loc.setNextNode(-1);
    locationList.push_back(loc);
    locationList[locationList[id].getPrevNode()].setNextNode(id);
    return id;
}

void LocationArray::updateLocation(int id, location loc, bool sameFlag)
{
    //ROS_INFO("updating %d, p %d -- l = %f || c = %f", id, loc.getPrevNode(), locationList[id].getDistance() ,loc.getDistance() );
    int prevID = locationList[id].getPrevNode();

    loc.setCurrNode(id);
    loc.setNextNode(-1);
    locationList[id] = loc;
    if(sameFlag)
    {
        locationList[id].setPrevNode(prevID);
    }
    locationList[locationList[id].getPrevNode()].setNextNode(id);
}

float LocationArray::getDistance(int id)
{
    return locationList[id].getDistance();
}

void LocationArray::setDistance(int id, float distance)
{
    locationList[id].setDistance(distance);
    return;
}

int LocationArray::searchLocation(float x, float y)
{
    for(int i=0;i<locationList.size();i++)
    {
        if(locationList[i].getX() == x && locationList[i].getY() == y)
            return locationList[i].getCurrNode();
    }
    return -1;
}

void LocationArray::eraseBugData(int id)
{
    this->locationList[id].eraseBugDetails();
}

void LocationArray::removeElement(int id)
{
//    for(int i=0;i< locationList.size();i++)
//    {
//        if(locationList[i].getID() == id)
//        {
//            locationList.erase(locationList.begin()+i);
//            break;
//        }
//    }
}
