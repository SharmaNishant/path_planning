#include <cstddef>
#include <iostream>
#include <math.h>
#include <path_planning/rrt.h>
#include <ros/ros.h>

using namespace rrt;

/**
 * default constructor for RRT class
 * initializes source to 0,0
 * adds sorce to rrtTree
 */
RRT::RRT() {
  RRT::rrtNode newNode;
  newNode.posX = 0;
  newNode.posY = 0;
  newNode.parentID = 0;
  newNode.nodeID = 0;
  rrtTree.push_back(newNode);
}

/**
 * default constructor for RRT class
 * initializes source to input X,Y
 * adds sorce to rrtTree
 */
RRT::RRT(double input_PosX, double input_PosY) {
  RRT::rrtNode newNode;
  newNode.posX = input_PosX;
  newNode.posY = input_PosY;
  newNode.parentID = 0;
  newNode.nodeID = 0;
  rrtTree.push_back(newNode);
}

/**
 * Returns the current RRT tree
 * @return RRT Tree
 */
vector<RRT::rrtNode> RRT::getTree() { return rrtTree; }

/**
 * For setting the rrtTree to the inputTree
 * @param rrtTree
 */
void RRT::setTree(vector<RRT::rrtNode> input_rrtTree) {
  rrtTree = input_rrtTree;
}

/**
 * to get the number of nodes in the rrt Tree
 * @return tree size
 */
int RRT::getTreeSize() { return rrtTree.size(); }

/**
 * adding a new node to the rrt Tree
 */
void RRT::addNewNode(RRT::rrtNode node) { rrtTree.push_back(node); }

/**
 * removing a node from the RRT Tree
 * @return the removed tree
 */
RRT::rrtNode RRT::removeNode(int id) {
  RRT::rrtNode tempNode = rrtTree[id];
  rrtTree.erase(rrtTree.begin() + id);
  return tempNode;
}

/**
 * getting a specific node
 * @param node id for the required node
 * @return node in the rrtNode structure
 */
RRT::rrtNode RRT::getNode(int id) { return rrtTree[id]; }

/**
 * return a node from the rrt tree nearest to the given point
 * @param X position in X cordinate
 * @param Y position in Y cordinate
 * @return nodeID of the nearest Node
 */
int RRT::getNearestNodeID(double X, double Y) {
  int i, returnID;
  double distance = 9999, tempDistance;
  for (i = 0; i < this->getTreeSize(); i++) {
    tempDistance = getEuclideanDistance(X, Y, getPosX(i), getPosY(i));
    if (tempDistance < distance) {
      distance = tempDistance;
      returnID = i;
    }
  }
  return returnID;
}

vector<int> RRT::getKNearestNodesID(double X, double Y, int k) {
  int i, returnID;
  double tempDistance;
  vector<int> nodeIDList;
  vector<double> distance;

  for (int i = 0; i < k; i++) {
    distance.push_back(9999);
    nodeIDList.push_back(-1);
  }
  int j;
  for (i = 0; i < this->getTreeSize(); i++) {
    j = k - 1;
    tempDistance = getEuclideanDistance(X, Y, getPosX(i), getPosY(i));
    while (j >= 0) {
      // if(distance[j] < tempDistance) break;
      if (j == 0 && distance[j] > tempDistance) {
        distance.insert(distance.begin(), tempDistance);
        nodeIDList.insert(nodeIDList.begin(), i);
        break;
      }
      if (tempDistance < distance[j] && tempDistance > distance[j - 1]) {
        distance.insert(distance.begin() + j, tempDistance);
        nodeIDList.insert(nodeIDList.begin() + j, i);
        break;
      }
      j--;
    }
    if (nodeIDList.size() > k)
      nodeIDList.erase(nodeIDList.begin() + k, nodeIDList.end());
    if (distance.size() > k)
      distance.erase(distance.begin() + k, distance.end());
  }
  // ROS_INFO("Returing %ld nodes", nodeIDList.size());
  return nodeIDList;
}

/**
 * returns X coordinate of the given node
 */
double RRT::getPosX(int nodeID) { return rrtTree[nodeID].posX; }

/**
 * returns Y coordinate of the given node
 */
double RRT::getPosY(int nodeID) { return rrtTree[nodeID].posY; }

/**
 * set X coordinate of the given node
 */
void RRT::setPosX(int nodeID, double input_PosX) {
  rrtTree[nodeID].posX = input_PosX;
}

/**
 * set Y coordinate of the given node
 */
void RRT::setPosY(int nodeID, double input_PosY) {
  rrtTree[nodeID].posY = input_PosY;
}

/**
 * returns parentID of the given node
 */
RRT::rrtNode RRT::getParent(int id) { return rrtTree[rrtTree[id].parentID]; }

/**
 * set parentID of the given node
 */
void RRT::setParentID(int nodeID, int parentID) {
  rrtTree[nodeID].parentID = parentID;
}

/**
 * add a new childID to the children list of the given node
 */
void RRT::addChildID(int nodeID, int childID) {
  rrtTree[nodeID].children.push_back(childID);
}

/**
 * returns the children list of the given node
 */
vector<int> RRT::getChildren(int id) { return rrtTree[id].children; }

/**
 * returns number of children of a given node
 */
int RRT::getChildrenSize(int nodeID) { return rrtTree[nodeID].children.size(); }

/**
 * returns euclidean distance between two set of X,Y coordinates
 */
double RRT::getEuclideanDistance(double sourceX, double sourceY,
                                 double destinationX, double destinationY) {
  return sqrt(pow(destinationX - sourceX, 2) + pow(destinationY - sourceY, 2));
}

/**
 * returns path from root to end node
 * @param endNodeID of the end node
 * @return path containing ID of member nodes in the vector form
 */
vector<int> RRT::getRootToEndPath(int endNodeID) {
  vector<int> path;
  path.push_back(endNodeID);
  while (rrtTree[path.front()].nodeID != 0) {
    // std::cout<<rrtTree[path.front()].nodeID<<endl;
    path.insert(path.begin(), rrtTree[path.front()].parentID);
  }
  return path;
}
