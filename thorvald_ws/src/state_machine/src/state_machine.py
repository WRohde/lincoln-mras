#!/usr/bin/python

import rospy
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

#greendetection subscriber and callback.
green_detection = False
def green_detection_callback(data):
    global green_detection 
    if(data.data =='TRUE'):
        green_detection = True
        print("green_detected")
    else:
        green_detection = False 
green_detection_sub = rospy.Subscriber("/green_detected",String,green_detection_callback)

def callSprayService():
    rospy.wait_for_service('/thorvald_001/spray')
    try:
        print("spray")
        callSpray = rospy.ServiceProxy('/thorvald_001/spray', Empty)
        return callSpray()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def launch(_):
    newState = 'ROAM'
    return(newState,_)

def roam(_):
    #TODO publish a random target position
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
    thorvald_StateMachine.run("")
    