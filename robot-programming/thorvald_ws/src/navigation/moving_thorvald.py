#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class move_and_avoid:
    def __init__(self,node_name,min_distance=3,avoidance_angle=0.7854):
        """
        min_distance is the minimum distance between the robot and an obstacle before it turns to avoid.
        avoidance_angle is the angle of the sector where collisions are checked.
        """
        rospy.init_node(node_name)
        self.pub = rospy.Publisher("/thorvald_001/twist_mux/cmd_vel",Twist,queue_size=0)
        self.sub = rospy.Subscriber("/thorvald_001/scan", LaserScan, self.laserscan_subscriber_callback)
        self.min_distance = min_distance
        self.avoidance_angle = avoidance_angle

    def laserscan_subscriber_callback(self,data):
        """
        processes the laserscan data and checks for collisions

        laserscan message type http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
        For the Thorvald Robot the laserscanner is positioned so that it sweeps from -pi/2 to pi/2
        """

        num_ranges_to_sample = int(self.avoidance_angle / data.angle_increment)
    
        sampled_ranges = data.ranges[int((len(data.ranges) -  num_ranges_to_sample)/2):int((len(data.ranges) +  num_ranges_to_sample)/2)]
        self.check_collisions(sampled_ranges,data.range_min) 
    
    def check_collisions(self,sampled_ranges,range_min):
        # the array of sampled ranges is split into left and right sides
        collisions = np.array(sampled_ranges) < self.min_distance 
        right_collisions = collisions[:int(len(collisions)/2)]
        left_collisions = collisions[int(len(collisions)/2):]

        if(np.sum(left_collisions) > np.sum(right_collisions)): # more collisions to the left so turn right
            self.publish_cmd_vel(ang=[0,0,-1.5708])  
        elif(np.sum(left_collisions) < np.sum(right_collisions)): #more collisions to the right so turn left
            self.publish_cmd_vel(ang=[0,0,1.5708])
        else: #keep going forwards
            self.publish_cmd_vel(lin=[1,0,0])

    def publish_cmd_vel(self,lin=[0,0,0],ang=[0,0,0]):
        message = Twist()
        message.linear.x = lin[0]
        message.linear.y = lin[1]
        message.linear.z = lin[2]
        message.angular.x = ang[0]
        message.angular.y = ang[1]
        message.angular.z = ang[2]
        self.pub.publish(message)

move_and_avoid = move_and_avoid("moving_thorvald")

while not rospy.is_shutdown():
    rospy.sleep(0.01)
