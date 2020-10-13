#!/usr/bin/env python

import rospy
import psutil
from std_msgs.msg import String

def publishFreeMem():
    pub = rospy.Publisher('freeMem', String, queue_size=10)
    rospy.init_node('publishFreeMem', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        freeMem = psutil.virtual_memory().free
        pub_str = "freeMem: {0}".format(freeMem)
        rospy.loginfo(pub_str)
        pub.publish(pub_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        publishFreeMem()
    except rospy.ROSInterruptException:
        pass
