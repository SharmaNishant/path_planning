//
// Created by nishant on 13/9/15.
//

#ifndef NISHANT_NCOREADER_H
#define NISHANT_NCOREADER_H

#include <geometry_msgs/Point.h>
#include <iostream>
#include <vector>

typedef geometry_msgs::Point Point;

using namespace std;

struct NCOLine {
  Point start;
  Point end;
};

// class for reading one non convex (NC) obstacle from a file
class NCOReader {
public:
  /** Default constructor */
  NCOReader(char *points, char *rects);

  /** Default destructor */
  virtual ~NCOReader() {}

  vector<NCOLine> getObstacleLines();
  vector<Point> getObstaclePoints();
  vector<vector<Point>> getObstacleRects();
  void setSourceGoalPoints(char *filename, Point &start, Point &goal);

protected:
private:
  // read one NC object from a file
  void readNCO(char *filename);
  void readNCORects(char *filename);

  // convert a NC object points into lines
  void convertToLines();
  vector<Point> obstaclePoints;
  vector<NCOLine> obstacleLines;
  vector<vector<Point>> obstacleRects;
};

#endif // NISHANT_NCOREADER_H
