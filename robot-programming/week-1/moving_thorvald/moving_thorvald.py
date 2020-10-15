#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class cmd_vel_talker:
    def __init__(self):
        rospy.init_node("moving_thorvald")
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
    cmd_vel_talker.publish(lin=[1,0,0],ang=[0,0,0.3927])
    rospy.sleep(0.05)
