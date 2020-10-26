#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class move_and_avoid:
    def __init__(self,node_name,min_distance=5,avoidance_angle=0.7854,forward_speed=0.5):
        """
        min_distance is the minimum distance between the robot and an obstacle before it turns to avoid.
        avoidance_angle is the angle of the sector where collisions are checked.
        """
        rospy.init_node(node_name)
        self.pub = rospy.Publisher("/thorvald_001/twist_mux/cmd_vel",Twist,queue_size=0)
        self.sub = rospy.Subscriber("/thorvald_001/scan", LaserScan, self.laserscan_subscriber_callback)
        self.min_distance = min_distance
        self.avoidance_angle = avoidance_angle
        self.detected_collisions = {"left":False,"right":False,"forward":False}
        self.forward_speed = forward_speed

    def __call__(self):
        """
        When called this function will move the robot forwards at self.forward_speed unless
        a collision is detected. The logic is based on the content of the dict 
        self.detected_collsions which should have a boolean value for the directions forward,
        left, and right.        
        """      
        rotational_speed = 0
        # if there are collisions detected in front stop. Otherwise move forwards
        if self.detected_collisions["forward"]:
            speed = 0
        else:
            speed = self.forward_speed

        #collisions in all directions, turn on the spot until a free direction is found.
        if self.detected_collisions["left"] and self.detected_collisions["right"] and self.detected_collisions["forward"]:
            print("collisions in all directions, turning left")
            rotational_speed = 1.5708
        #collisions on the left, free on the right so turn right.   
        if self.detected_collisions["left"] and not self.detected_collisions["right"]:
            print("collisions on the left, clear on the right. Turning right") 
            rotational_speed = -1.5708
        #collisions on the right, free on the left so turn left
        if not self.detected_collisions["left"] and self.detected_collisions["right"]: 
            print("collisions on the righ, clear on the left. Turning left") 
            rotational_speed = 1.5708

        self.publish_cmd_vel(speed=speed, rotational_speed=rotational_speed)

    def laserscan_subscriber_callback(self,data):
        """
        processes the laserscan data and checks for collisions

        laserscan message type http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
        For the Thorvald Robot the laserscanner is positioned so that it sweeps from -pi/2 to pi/2
        """

        # ranges from the laserscan data are sampled in a sector avoidance_angle wide
        num_ranges_to_sample = int(self.avoidance_angle / data.angle_increment)
        sampled_ranges = data.ranges[int((len(data.ranges) -  num_ranges_to_sample)/2):int((len(data.ranges) +  num_ranges_to_sample)/2)]
        self.check_collisions(sampled_ranges)
    
    def check_collisions(self,sampled_ranges):
        # the array of sampled ranges is split into left and right sides
        collisions = np.array(sampled_ranges) < self.min_distance 
        detected_collisions = {"left":False,"right":False,"forward":False}

        #check for collisions on the right
        if np.sum(collisions[:int(len(collisions)/4)]) > 0:
            detected_collisions["left"]=True

        #check for collisions in forward
        if np.sum(collisions[int(len(collisions)/4):int(len(collisions)*3/4)]) > 0:
            detected_collisions["forward"]=True

        #check for collisions on the left
        if np.sum(collisions[int(len(collisions)*3/4):]) > 0:
            detected_collisions["right"]=True

        self.detected_collisions = detected_collisions

    def publish_cmd_vel(self,speed=0,rotational_speed=0):
        message = Twist()
        message.linear.x = speed
        message.angular.z = rotational_speed
        self.pub.publish(message)

move_and_avoid = move_and_avoid("moving_thorvald")

while not rospy.is_shutdown():
    move_and_avoid()
    rospy.sleep(0.1)
