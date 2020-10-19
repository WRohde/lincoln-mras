#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class move_and_avoid:
    def __init__(self,node_name,min_distance=3):
        rospy.init_node(node_name)
        self.pub = rospy.Publisher("/thorvald_001/twist_mux/cmd_vel",Twist,queue_size=0)
        self.min_distance = min_distance

    def laserscan_subscriber_callback(self,data):
        """
        These are the objects available in the laserscan message data 
        Source: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
        float32 angle_min        # start angle of the scan [rad]
        float32 angle_max           # end angle of the scan [rad]
        float32 angle_increment  # angular distance between measurements [rad]
        float32 time_increment   # time between measurements [seconds] - if your scanner
                                # is moving, this will be used in interpolating position
                                # of 3d points
        float32 scan_time        # time between scans [seconds]
        float32 range_min        # minimum range value [m]
        float32 range_max        # maximum range value [m]
        float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
        float32[] intensities    # intensity data [device-specific units].  If your
                                # device does not provide intensities, please leave
                                # the array empty.
        """
        
        centre_range = data.ranges[int(len(data.ranges)/2)]
        
        # if the robot will collide turn, otherwise go straight
        if(centre_range < self.min_distance):
            self.publish_cmd_vel(ang=[0,0,1.5708])
        else:
            self.publish_cmd_vel()
    
    def laserscan_subscriber(self):
        self.sub = rospy.Subscriber("/thorvald_001/scan", LaserScan, self.laserscan_subscriber_callback)

    def publish_cmd_vel(self,lin=[1,0,0],ang=[0,0,0]):
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
    move_and_avoid.laserscan_subscriber()
    rospy.sleep(0.05)
