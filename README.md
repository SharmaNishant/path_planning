# Path Planning

Offline path planners (RRT, RRT*, Visibility Graphs, Bug_Flood) 

* RRT and RRT* - Rapidly Exploring Random Trees
* Visibility Graphs - Implementation of Visibility graph is O(n^3).
* Bug_Flood - A bug flood algorithm for obstacle rich environment

A camparison between some offline path planning algorithms. Implmentation is done in ROS and visulization is done using RVIZ. 

**Update:** Working on a newer implementation of the bug_flood.  

## Installation

~~~ bash
# setting up workspace
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace

# download this repository
git clone https://github.com/SharmaNishant/path_planning

# build the package
cd ..
catkin_make
~~~

