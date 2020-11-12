#!/usr/bin/python

import rospy
import sys
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_srvs.srv import Empty


class StateMachine:
    """
    state machine class from the example here: https://www.python-course.eu/finite_state_machine.php
    Modified for ROS
    """
    def __init__(self):
        self.handlers = {}
        self.startState = None
        self.endStates = []

    def add_state(self, name, handler, end_state=0):
        name = name.upper()
        self.handlers[name] = handler
        if end_state:
            self.endStates.append(name)

    def set_start(self, name):
        self.startState = name.upper()

    def run(self, cargo):
        try:
            handler = self.handlers[self.startState]
        except:
            raise InitializationError("must call .set_start() before .run()")
        #if not self.endStates:
        #    raise  InitializationError("at least one state must be an end_state")
    
        while not rospy.is_shutdown():
            (newState, cargo) = handler(cargo)
            if newState.upper() in self.endStates:
                print("reached ", newState)
                break 
            else:
                handler = self.handlers[newState.upper()]
            rospy.sleep(0.1)

#greendetection callback.
green_detection = False
def green_detection_callback(data):
    global green_detection 
    if(data.data =='TRUE'):
        green_detection = True
    else:
        green_detection = False 

def callSprayService():
    rospy.wait_for_service('/thorvald_001/spray')
    try:
        callSpray = rospy.ServiceProxy('/thorvald_001/spray', Empty)
        return callSpray()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

#State machine states
def launch(_):
    newState = 'ROAM'
    return(newState,_)

def roam(_):
    #publish a random target position
    target_position = Point()
    target_position_pub.publish(target_position)
    
    if(green_detection == True):
        newState = 'SPRAY'
    else:
        newState = 'ROAM'
    return(newState,_)

def spray(_):
    callSprayService()
    green_detection = False
    newState = 'ROAM'
    return(newState,_)

#setting up the state machine  
thorvald_StateMachine = StateMachine() 
thorvald_StateMachine.add_state('LAUNCH',launch)
thorvald_StateMachine.set_start('LAUNCH')
thorvald_StateMachine.add_state('ROAM',roam)
thorvald_StateMachine.add_state('SPRAY',spray)

rospy.init_node("thorvald_state_machine",anonymous=True)

if __name__ == '__main__':
    if(len(sys.argv)>1):
        robot_name = sys.argv[1]
        print(robot_name)
    else:
        robot_name = "thorvald_001"

    green_detection_sub = rospy.Subscriber("/green_detected",String,green_detection_callback)
    #target position publisher. The moving_thorvald node subscribes to these messages and the robot will navigate to them.
    target_position_pub = rospy.Publisher("/{}/target_position".format(robot_name),Point,queue_size=0)

    thorvald_StateMachine.run("")
    