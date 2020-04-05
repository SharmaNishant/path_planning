#ifndef BUG_H
#define BUG_H

#include <vector>

#define movingToGoal 0
#define boundaryFollowing 1
#define reachedEnd 2

using namespace std;

struct location {
  double x;
  double y;
};

class bug {
public:
  bug();
  bug(int state, double x, double y, vector<location> path);
  ~bug();

  void setState(int state);
  void setPosX(double x);
  void setPosY(double y);
  void setLastX(double x);
  void setLastY(double y);
  void setStepSize(double sSize);
  void setBoundaryID(int boundaryId);
  void setBoundaryIDIndex(int index);
  void setMoveToGoalFlag(bool flag);

  int getState();
  double getPosX();
  double getPosY();
  double getLastX();
  double getLastY();
  double getStepSize();
  int getBoundaryID();
  int getBoundaryIDIndex();
  bool getMoveToGoalFlag();

  void setPath(vector<location> path);
  vector<location> getPath();
  void addNodeToPath(location newNode);

  double getPathLength();

protected:
private:
  int state;

  double posX;
  double posY;
  double lastX;
  double lastY;

  double stepSize;

  bool moveToGoalFlag;

  int boundaryID;
  int boundaryIDIndex;

  vector<location> path;
};

#endif // BUG_H
