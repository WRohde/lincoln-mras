#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import numpy as np

class cmd_vel_talker:
    def __init__(self):
        rospy.init_node("random_walk")
        self.pub = rospy.Publisher("/thorvald_001/twist_mux/cmd_vel",Twist,queue_size=0)

    def publish(self,lin=[0,0,0],ang=[0,0,0]):
        message = Twist()
        message.linear.x = lin[0]
        message.linear.y = lin[1]
        message.linear.z = lin[2]
        message.angular.x = ang[0]
        message.angular.y = ang[1]
        message.angular.z = ang[2]
        self.pub.publish(message)

cmd_vel_talker = cmd_vel_talker()

while not rospy.is_shutdown():
    linear_vector = [1,0,0] 
    angular_vector = [0,0,np.random.rand(1)*2*np.pi]
    cmd_vel_talker.publish(lin=linear_vector,ang=angular_vector)
    rospy.sleep(1)
