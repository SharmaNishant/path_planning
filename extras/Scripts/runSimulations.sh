#!/bin/bash
#for (( i=1; i<=100; i++ ))
#do
#   echo $i
# i=30
#   rosrun path_planning bug_node ~/catkin_ws/src/path_planning/extras/10_obstacle/data_$i.txt ~/catkin_ws/src/path_planning/extras/10_obstacle/sorGoal_$i.txt
#   rosrun path_planning vg_node ~/catkin_ws/src/path_planning/extras/10_obstacle/data_$i.txt ~/catkin_ws/src/path_planning/extras/10_obstacle/sorGoal_$i.txt
#   rosrun path_planning rrt_node ~/catkin_ws/src/path_planning/extras/10_obstacle/data_$i.txt ~/catkin_ws/src/path_planning/extras/10_obstacle/sorGoal_$i.txt
#   rosrun path_planning rrt_star_node ~/catkin_ws/src/path_planning/extras/10_obstacle/data_$i.txt ~/catkin_ws/src/path_planning/extras/10_obstacle/sorGoal_$i.txt
#done

#for (( i=1; i<=100; i++ ))
#do
#   echo $i
#i=55
#   rosrun path_planning bug_node ~/catkin_ws/src/path_planning/extras/25_obstacle/data_$i.txt ~/catkin_ws/src/path_planning/extras/25_obstacle/sorGoal_$i.txt
#   rosrun path_planning vg_node ~/catkin_ws/src/path_planning/extras/25_obstacle/data_$i.txt ~/catkin_ws/src/path_planning/extras/25_obstacle/sorGoal_$i.txt
#   rosrun path_planning rrt_node ~/catkin_ws/src/path_planning/extras/25_obstacle/data_$i.txt ~/catkin_ws/src/path_planning/extras/25_obstacle/sorGoal_$i.txt
#   rosrun path_planning rrt_star_node ~/catkin_ws/src/path_planning/extras/25_obstacle/data_$i.txt ~/catkin_ws/src/path_planning/extras/25_obstacle/sorGoal_$i.txt

#done

#for (( i=1; i<=100; i++ ))
#do
#   echo $i
#i=24
#   rosrun path_planning bug_node ~/catkin_ws/src/path_planning/extras/50_obstacle/data_$i.txt ~/catkin_ws/src/path_planning/extras/50_obstacle/sorGoal_$i.txt
#   rosrun path_planning vg_node ~/catkin_ws/src/path_planning/extras/50_obstacle/data_$i.txt ~/catkin_ws/src/path_planning/extras/50_obstacle/sorGoal_$i.txt
#   rosrun path_planning rrt_node ~/catkin_ws/src/path_planning/extras/50_obstacle/data_$i.txt ~/catkin_ws/src/path_planning/extras/50_obstacle/sorGoal_$i.txt
#   rosrun path_planning rrt_star_node ~/catkin_ws/src/path_planning/extras/50_obstacle/data_$i.txt ~/catkin_ws/src/path_planning/extras/50_obstacle/sorGoal_$i.txt
#done

#for (( i=1; i<=100; i++ ))
#do
#   echo $i
#   rosrun path_planning rrt_star_node ~/catkin_ws/src/path_planning/extras/100_obstacle/data_$i.txt ~/catkin_ws/src/path_planning/extras/100_obstacle/sorGoal_$i.txt
#done


#it starts here 250 obstacles


for (( i=1; i<=100; i++ ))
do
   echo $i
   rosrun path_planning bug_node ~/catkin_ws/src/path_planning/extras/250_obstacle/data_$i.txt ~/catkin_ws/src/path_planning/extras/250_obstacle/sorGoal_$i.txt
done


for (( i=1; i<=100; i++ ))
do
   echo $i
   rosrun path_planning vg_node ~/catkin_ws/src/path_planning/extras/250_obstacle/data_$i.txt ~/catkin_ws/src/path_planning/extras/250_obstacle/sorGoal_$i.txt
done


for (( i=1; i<=100; i++ ))
do
   echo $i
   rosrun path_planning rrt_node ~/catkin_ws/src/path_planning/extras/250_obstacle/data_$i.txt ~/catkin_ws/src/path_planning/extras/250_obstacle/sorGoal_$i.txt
done

for (( i=1; i<=100; i++ ))
do
   echo $i
   rosrun path_planning rrt_star_node ~/catkin_ws/src/path_planning/extras/250_obstacle/data_$i.txt ~/catkin_ws/src/path_planning/extras/250_obstacle/sorGoal_$i.txt
done
