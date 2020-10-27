#!/usr/bin/python3
import numpy as np

robot_coord = np.random.random(2)
cow_coord = np.random.random(2)
cow_wrt_robot = cow_coord - robot_coord

print("robot_coord is {0}, cow_coord is {1}".format(robot_coord,cow_coord))
print("The cow is at coords {} in the robot coordinate system".format(cow_wrt_robot))

#trig method
theta = np.arctan2(cow_coord[1]-robot_coord[1],cow_coord[0]-robot_coord[0])
cow_robot_norm = np.linalg.norm(cow_wrt_robot)
print("through trig")
print("The angle between the robot and cow is {}, the distance is {}".format(theta, cow_robot_norm))

#matrix method
