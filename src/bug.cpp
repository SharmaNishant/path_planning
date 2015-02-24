#include "path_planning/bug.h"

bug::bug()
{
    this->stepSize = 1;
    //ctor
}

bug::bug(int state, double x, double y, vector<location> path)
{
    this->stepSize = 1;
	this->state = state;
	this->posX = x;
	this->posY = y;
	this->path = path;
}

bug::~bug()
{
    //dtor
}

void bug::setState(int state)
{
	this->state = state;
}

void bug::setPosX(double x)
{
    this->posX = x;
}

void bug::setPosY(double y)
{
    this->posY = y;
}

void bug::setLastX(double x)
{
    this->lastX = x;
}

void bug::setLastY(double y)
{
    this->lastX = y;
}

int bug::getState()
{
    return this->state;
}

double bug::getPosX()
{
    return this->posX;
}

double bug::getPosY()
{
    return this->posY;
}

double bug::getLastX()
{
    return this->lastX;
}

double bug::getLastY()
{
    return this->lastY;
}

void bug::setBoundaryID(int boundaryId)
{
    this->boundaryID = boundaryId;
}

void bug::setBoundaryIDIndex(int index)
{
    this->boundaryIDIndex = index;
}

int bug::getBoundaryID()
{
    return this->boundaryID;
}

int bug::getBoundaryIDIndex()
{
    this->boundaryIDIndex;
}

void bug::setPath(vector<location> path)
{
    this->path = path;
}

vector<location> bug::getPath()
{
    return this->path;
}

void bug::addNodeToPath(location newNode)
{
    this->path.push_back(newNode);
}

double bug::getPathLength()
{
    return this->path.size();
}


void bug::setStepSize(double sSize)
{
    this->stepSize = sSize;
}

double bug::getStepSize()
{
    return this->stepSize;
}


void bug::setMoveToGoalFlag(bool flag)
{
    this->moveToGoalFlag = flag;
}

bool bug::getMoveToGoalFlag()
{
    return this->moveToGoalFlag;
}



