#!/bin/bash
for (( i=1; i<=100; i++ ))
do
   rosrun path_planning bug_node ~/catkin_ws/src/path_planning/extras/10_obstacle/data_$i.txt ~/catkin_ws/src/path_planning/extras/10_obstacle/sorGoal_$i.txt
done

for (( i=1; i<=100; i++ ))
do
   rosrun path_planning bug_node ~/catkin_ws/src/path_planning/extras/25_obstacle/data_$i.txt ~/catkin_ws/src/path_planning/extras/25_obstacle/sorGoal_$i.txt
done

for (( i=1; i<=100; i++ ))
do
   rosrun path_planning bug_node ~/catkin_ws/src/path_planning/extras/50_obstacle/data_$i.txt ~/catkin_ws/src/path_planning/extras/50_obstacle/sorGoal_$i.txt
done

for (( i=1; i<=100; i++ ))
do
   rosrun path_planning bug_node ~/catkin_ws/src/path_planning/extras/100_obstacle/data_$i.txt ~/catkin_ws/src/path_planning/extras/100_obstacle/sorGoal_$i.txt
done

for (( i=1; i<=100; i++ ))
do
   rosrun path_planning bug_node ~/catkin_ws/src/path_planning/extras/250_obstacle/data_$i.txt ~/catkin_ws/src/path_planning/extras/250_obstacle/sorGoal_$i.txt
done
