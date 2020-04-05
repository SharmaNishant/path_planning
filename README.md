**NOTE:** This repository is not maintained anymore. If you have any questions
regarding this work, please reach out to the author.

# Path Planning

Offline path planners (RRT, RRT*, Visibility Graphs, Bug_Flood)

* RRT and RRT* - Rapidly Exploring Random Trees
* Visibility Graphs - Implementation of Visibility graph is O(n^3).
* Bug Flood - A bug flood algorithm for obstacle rich environment

A comparison between some off-line path planning algorithms.
Implementation is done in ROS and visualization is done using RVIZ.

## Installation

Last tested with ROS1, Ubuntu 14.04, and catkin.

```bash
# setting up workspace
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace

# download this repository
git clone https://github.com/SharmaNishant/path_planning

# build the package
cd ..
catkin_make
```

