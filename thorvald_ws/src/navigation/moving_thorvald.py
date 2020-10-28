#!/usr/bin/python

import sys
import rospy
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan

class move_and_avoid:
    def __init__(self,robot_name,min_distance=5,avoidance_angle=0.7854,forward_speed=2,publish_closest_collision=True):
        """
        robot_name should be in the format "thorvald_001", "thorvald_002", etc so that topics are published correctly
        min_distance is the minimum distance between the robot and an obstacle before it turns to avoid.
        avoidance_angle is the angle of the sector where collisions are checked.
        """
        rospy.init_node("moving_"+robot_name,anonymous=True)
        self.pub = rospy.Publisher("/{}/twist_mux/cmd_vel".format(robot_name),Twist,queue_size=0)
        self.sub = rospy.Subscriber("/{}/scan".format(robot_name), LaserScan, self.laserscan_subscriber_callback)
        self.publish_closest_collision = publish_closest_collision
        if (self.publish_closest_collision):
            self.closest_collision_publisher = rospy.Publisher("/{}/closest_collision".format(robot_name), PoseStamped,queue_size=0)

        self.robot_name = robot_name
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
        #if there are collisions detected in front stop. Otherwise move forwards
        if self.detected_collisions["forward"]:
            #if the robot is blocked in front it has to turn on the spot. 
            rotational_speed = 1.5708
            speed = 0
        else:
            speed = self.forward_speed

        #collisions detected on the left, no collisions on the right. So turn right.   
        if self.detected_collisions["left"] and not self.detected_collisions["right"]:
            rotational_speed = -1.5708
        #collisions detected on the right, no collisions on the left. So turn left
        if not self.detected_collisions["left"] and self.detected_collisions["right"]:
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

        if self.publish_closest_collision:
            #find the polar coord for the minimum distance
            min_distance = min(data.ranges)
            min_distance_angle = data.angle_increment * data.ranges.index(min_distance) - np.pi/2

            #convert to a pose WRT /robot_name/xxxx frame
            pose = PoseStamped()
            pose.header.frame_id = "{}/hokuyo".format(self.robot_name)
            pose.pose.position.x = min_distance * np.cos(min_distance_angle)
            pose.pose.position.y = min_distance * np.sin(min_distance_angle)
            #convert angle to quaternion
            r = Rotation.from_euler('z', min_distance_angle)
            min_distance_quaternion = r.as_quat()
            pose.pose.orientation.x = min_distance_quaternion[0]
            pose.pose.orientation.y = min_distance_quaternion[1]
            pose.pose.orientation.z = min_distance_quaternion[2]
            pose.pose.orientation.w = min_distance_quaternion[3]

            #publish pose
            self.closest_collision_publisher.publish(pose)

    
    def check_collisions(self,sampled_ranges):
        """
        The sampled_ranges array is split into three areas in relation to the robot: front, left, right
        if collisions are detected in each area the entry in dict self.detected_collisions for the area 
        is set true.
        """
        # the array of sampled ranges is split into left and right sides
        collisions = np.array(sampled_ranges) < self.min_distance 
        detected_collisions = {"left":False,"right":False,"forward":False}

        #check for collisions on the right
        if np.sum(collisions[:int(len(collisions)/4)]) > 0:
            detected_collisions["right"]=True

        #check for collisions in forward
        if np.sum(collisions[int(len(collisions)/4):int(len(collisions)*3/4)]) > 0:
            detected_collisions["forward"]=True

        #check for collisions on the left
        if np.sum(collisions[int(len(collisions)*3/4):]) > 0:
            detected_collisions["left"]=True

        self.detected_collisions = detected_collisions       

    def publish_cmd_vel(self,speed=0,rotational_speed=0):
        message = Twist()
        message.linear.x = speed
        message.angular.z = rotational_speed
        self.pub.publish(message)

if __name__ == '__main__':
    if(len(sys.argv)>1):
        robot_name = sys.argv[1]
    else:
        robot_name = "thorvald_001"
    
    move_and_avoid = move_and_avoid(robot_name)
    
    while not rospy.is_shutdown():
        move_and_avoid()
        rospy.sleep(0.1)

