#!/usr/bin/python

import sys
import rospy
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Twist, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import tf

class move:
    def __init__(self,robot_name,min_distance=2,max_forward_speed=2,publish_closest_collision=True):
        """
        robot_name should be in the format "thorvald_001", "thorvald_002", etc so that topics are published correctly
        min_distance is the minimum distance between the robot and an obstacle before it turns to avoid.
        """

        self.robot_name = robot_name
        self.min_distance = min_distance
        self.max_forward_speed = max_forward_speed

        rospy.init_node("moving_"+robot_name,anonymous=True)
        
        #subscribers
        self.scan_sub = rospy.Subscriber("/{}/scan".format(robot_name), LaserScan, self.laserscan_subscriber_callback)
        self.odometry_sub = rospy.Subscriber("/{}/odometry/gazebo".format(robot_name),Odometry, self.odometry_subscriber_callback)
        self.target_sub = rospy.Subscriber("/{}/target_position".format(robot_name),PoseStamped,self.move_to_position)
        self.state = ""
        self.robot_state = rospy.Subscriber("/{}/state".format(robot_name),String,self.state_callback)
        self.transforms = tf.TransformListener()
        
        #publishers
        self.pub = rospy.Publisher("/{}/twist_mux/cmd_vel".format(robot_name),Twist,queue_size=0)
        #move_status is used to control the behaviour of the robot. There are currently  
        self.move_status = ""
        self.move_status_pub = rospy.Publisher("/{}/move_status".format(robot_name),String,queue_size=0)
        #closest_collision message is a point indicating location of closest collision for Rviz
        self.publish_closest_collision = publish_closest_collision
        if (self.publish_closest_collision):
            self.closest_collision_publisher = rospy.Publisher("/{}/closest_collision".format(robot_name), PoseStamped,queue_size=0)

        #defining some variables for use in later functions
        self.detected_collisions = {"left":False,"right":False,"forward":False}
        self.position = Pose()
        self.rotation = Quaternion()

    def move_to_position(self,data):
        """
        this function will attempt to move the robot to within a threshold of a given position and stop there.
        """        

        position_threshold = 0.1
        angular_threshold = 0.1

        cmd_vel_message = Twist()
        
        # transform errors from world frame to robot frame.
        # http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29                                                                                           
        try:
            self.transforms.waitForTransform('/{}/base_link'.format(robot_name), data.header.frame_id,rospy.Time(),rospy.Duration(0.5))
            target_position_WRT_base = self.transforms.transformPose('/{}/base_link'.format(robot_name), data)
            position_error_WRT_base = np.array([target_position_WRT_base.pose.position.x,\
                    target_position_WRT_base.pose.position.y,\
                    target_position_WRT_base.pose.position.z])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return -1
        
        #TODO rotate so the target_position is in the forwards direction

        #move to target in x and stop within threshold distance of it.
        if abs(position_error_WRT_base[0]) > position_threshold:                                                                                                                                                                                                         
            cmd_vel_message.linear.x = np.sign(position_error_WRT_base[0]) * min(abs(position_error_WRT_base[0]), self.max_forward_speed)
        #if x is within threshold, move to target in y and stop within threshold distance of it.
        if abs(position_error_WRT_base[1]) > position_threshold:
            cmd_vel_message.linear.y = np.sign(position_error_WRT_base[1]) * min(abs(position_error_WRT_base[1]), self.max_forward_speed)

        self.move_status = "MOVING"
        self.move_status_pub.publish(self.move_status)
        self.pub.publish(cmd_vel_message)

    def state_callback(self,data):
        """
        Makes the state_machine state for the robot accessible to the rest of the class.
        """        
        self.state = data

    def odometry_subscriber_callback(self,data):
        """
        Makes the odometry data accessible to the rest of the class.
        """
        self.position = data.pose.pose.position
        self.rotation = data.pose.pose.orientation

    def laserscan_subscriber_callback(self,data):
        """
        processes the laserscan data and checks for nearby collisions

        laserscan message type http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
        For the Thorvald Robot the laserscanner is positioned so that it sweeps from -pi/2 to pi/2
        """

        min_scan_distance = min(data.ranges)
        if(min_scan_distance < self.min_distance):
            self.move_status = "STATIONARY:COLLISION" 
            self.move_status_pub.publish(self.move_status)

        if self.publish_closest_collision:
            #find the polar coord for the minimum distance
            min_scan_distance = min(data.ranges)
            min_scan_distance_angle = data.angle_increment * data.ranges.index(min_scan_distance) - data.angle_min

            #convert to a pose WRT /robot_name/hokuyo frame
            pose = PoseStamped()
            pose.header.frame_id = data.header.frame_id
            pose.pose.position.x = min_scan_distance * np.cos(min_scan_distance_angle)
            pose.pose.position.y = min_scan_distance * np.sin(min_scan_distance_angle)
            #convert angle to quaternion
            r = Rotation.from_euler('z', min_scan_distance_angle)
            min_scan_distance_quaternion = r.as_quat()
            pose.pose.orientation.x = min_scan_distance_quaternion[0]
            pose.pose.orientation.y = min_scan_distance_quaternion[1]
            pose.pose.orientation.z = min_scan_distance_quaternion[2]
            pose.pose.orientation.w = min_scan_distance_quaternion[3]

            #publish pose
            self.closest_collision_publisher.publish(pose)  

if __name__ == '__main__':
    if(len(sys.argv)>1):
        robot_name = sys.argv[1]
    else:
        robot_name = "thorvald_001"
    
    move = move(robot_name)
    
    while not rospy.is_shutdown():
        rospy.sleep(0.1)

