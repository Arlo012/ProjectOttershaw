#!/usr/bin/env python
## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import os
from std_msgs.msg import String

from args import args

dir = os.path.dirname(__file__)

#Data to hold
val1 = 0

def listener(): 
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('servo', String, servoCall)
    
    rospy.spin()


def servoCall(data):
    '''
    Do work for this particular call using 'data'
    
    Inputs:
        data = generic value (string, int, etc) for processing
    '''
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    val1 = data
    
    #Potential for return call here via publishing a new message


if __name__ == '__main__':
  listener()
  
