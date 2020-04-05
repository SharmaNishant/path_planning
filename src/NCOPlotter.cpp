#include <path_planning/NCOReader.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

void initializeMarkers(visualization_msgs::Marker &boundary,
                       visualization_msgs::Marker &obstacle,
                       visualization_msgs::Marker &startPoint,
                       visualization_msgs::Marker &endPoint) {
  // init headers
  boundary.header.frame_id = obstacle.header.frame_id =
      startPoint.header.frame_id = endPoint.header.frame_id = "path_planner";
  boundary.header.stamp = obstacle.header.stamp = startPoint.header.stamp =
      endPoint.header.stamp = ros::Time::now();
  boundary.ns = "path_planner_bound";
  obstacle.ns = "path_planner_obs";
  startPoint.ns = "path_planner_start";
  endPoint.ns = "path_planner_end";
  boundary.action = obstacle.action = startPoint.action = endPoint.action =
      visualization_msgs::Marker::ADD;
  boundary.pose.orientation.w = obstacle.pose.orientation.w =
      startPoint.pose.orientation.w = endPoint.pose.orientation.w = 1.0;

  // setting id for each marker
  boundary.id = 100;
  obstacle.id = 101;
  startPoint.id = 111;
  endPoint.id = 110;

  // defining types
  boundary.type = visualization_msgs::Marker::LINE_STRIP;
  obstacle.type = visualization_msgs::Marker::LINE_LIST;
  startPoint.type = visualization_msgs::Marker::SPHERE;
  endPoint.type = visualization_msgs::Marker::SPHERE;

  // setting scale
  boundary.scale.x = 1;
  obstacle.scale.x = 1;

  startPoint.scale.x = 2;
  startPoint.scale.y = 2;
  startPoint.scale.z = 2;

  endPoint.scale.x = 2;
  endPoint.scale.y = 2;
  endPoint.scale.z = 2;

  // assigning colors
  boundary.color.r = obstacle.color.r = 0.0f;
  boundary.color.g = obstacle.color.g = 0.0f;
  boundary.color.b = obstacle.color.b = 0.0f;

  startPoint.color.g = startPoint.color.a = 1.0f;
  endPoint.color.r = endPoint.color.a = 1.0f;

  boundary.color.a = obstacle.color.a = 1.0f;
}

vector<Point> initializeBoundary() {
  vector<geometry_msgs::Point> bondArray;

  geometry_msgs::Point point;

  // first point
  point.x = 0;
  point.y = 0;
  point.z = 0;

  bondArray.push_back(point);

  // second point
  point.x = 0;
  point.y = 100;
  point.z = 0;

  bondArray.push_back(point);

  // third point
  point.x = 100;
  point.y = 100;
  point.z = 0;

  bondArray.push_back(point);

  // fourth point
  point.x = 100;
  point.y = 0;
  point.z = 0;
  bondArray.push_back(point);

  // first point again to complete the box
  point.x = 0;
  point.y = 0;
  point.z = 0;
  bondArray.push_back(point);

  return bondArray;
}

vector<Point> initializeObstacle(NCOReader &ncoReader) {
  vector<Point> obstacleLines;
  vector<NCOLine> obstacleNCOLines = ncoReader.getObstacleLines();

  // make all lines for visual
  for (int i = 0; i < obstacleNCOLines.size(); i++) {
    obstacleLines.push_back(obstacleNCOLines[i].start);
    obstacleLines.push_back(obstacleNCOLines[i].end);
  }
  std::cout << "No. of lines: " << obstacleLines.size() / 2 << std::endl;
  return obstacleLines;
}

int main(int argc, char **argv) {
  // initializing ROS
  ros::init(argc, argv, "env_node");
  ros::NodeHandle n;

  // defining Publisher
  ros::Publisher env_publisher =
      n.advertise<visualization_msgs::Marker>("path_planner_nc_bug", 1);

  // defining markers
  visualization_msgs::Marker boundary;
  visualization_msgs::Marker obstacle;
  visualization_msgs::Marker startPoint;
  visualization_msgs::Marker endPoint;

  initializeMarkers(boundary, obstacle, startPoint, endPoint);

  NCOReader ncoReader(argv[1], argv[2]);

  boundary.points = initializeBoundary();
  obstacle.points = initializeObstacle(ncoReader);

  ncoReader.setSourceGoalPoints(argv[3], startPoint.pose.position,
                                endPoint.pose.position);

  env_publisher.publish(boundary);
  env_publisher.publish(obstacle);
  env_publisher.publish(startPoint);
  env_publisher.publish(endPoint);

  while (ros::ok()) {
    env_publisher.publish(boundary);
    ros::spinOnce();
    ros::Duration(0.1).sleep();

    env_publisher.publish(obstacle);
    ros::spinOnce();
    ros::Duration(0.1).sleep();

    env_publisher.publish(startPoint);
    ros::spinOnce();
    ros::Duration(0.1).sleep();

    env_publisher.publish(endPoint);
    ros::spinOnce();
    ros::Duration(0.1).sleep();

    ros::Duration(1).sleep();
  }
  return 1;
}
