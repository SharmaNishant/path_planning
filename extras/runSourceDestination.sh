#!/bin/bash
for (( i=1; i<=100; i++ ))
do
   ./a.out ./10_obstacle/data_$i.txt ./10_obstacle/sorGoal_$i.txt
done

for (( i=1; i<=100; i++ ))
do
   ./a.out ./25_obstacle/data_$i.txt ./25_obstacle/sorGoal_$i.txt
done

for (( i=1; i<=100; i++ ))
do
   ./a.out ./50_obstacle/data_$i.txt ./50_obstacle/sorGoal_$i.txt
done

for (( i=1; i<=100; i++ ))
do
   ./a.out ./100_obstacle/data_$i.txt ./100_obstacle/sorGoal_$i.txt
done

for (( i=1; i<=100; i++ ))
do
   ./a.out ./250_obstacle/data_$i.txt ./250_obstacle/sorGoal_$i.txt
done
