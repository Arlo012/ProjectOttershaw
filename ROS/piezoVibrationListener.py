#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import os
from std_msgs.msg import Int32

class SpiderLegs:
    '''
    Contains three arrays:
        indexes of legs on ground
        indexes of legs in movement
        indexes of legs that collided

    Could be expanded for other processing features
    such as position of each leg in 3D space
    '''
    def __init__(self):
        self.legsOnGround = []
        self.legsInMovement = []
        self.legsCollided = []

    #These functions are used for debugging
    #Could access the arrays directly once implementation is complete
    def getLegsOnGround():
        print legsOnGround
        print "-----------"

    def getLegsInMovement():
        print legsInMovement
        print "-----------"

    def getlegsCollided():
        print legsCollided
        print "-----------"

borisLegs = SpiderLegs()
legStatus = []  #Create array of 0's, 1's and -1's from incomming readings
numReadings = 0 #keep count of readings to divide data in blocks of 8 (one per leg)

def listener(): 
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaneously.
    rospy.init_node('LegPiezo', anonymous=True)

    rospy.Subscriber('piezo', Int32, piezoCall)
    
    rospy.spin()


def piezoCall(data):
    '''
    Do work for this particular call using 'data'
    This function is called every time a new data comes in on the listener.

    Inputs:
        data = array of -1's, 1's & 0's,

    Value determines if the leg's on the ground (0), in the air (1), or collided (-1)
    '''

   #TODO: Test if leg in movement generates enough vibration to be detected by sensor
   #to use inverted values, i.e. using 0 for completed step and 1 for leg in movement
   
   numReadings = numReadings + 1

   if numReadings % 8 != 0:
        legStatus.append(data.data)
   else:
      for i, leg in enumerate(legStatus):
        if leg == 0:
            borisLegs.legsOnGround.append(i)
        elif leg == 1:
            borisLegs.legsInMovement.append(i)
        elif leg == -1:
            borisLegs.legsCollided.append(i)
        else:
            print "Invalid Status on leg %s" % i

        #Do we need to call the function every time a new data passes through ROS?
        #Would we actuate the servos from this python script? - would need a servo publisher.
        borisLegs.getLegsOnGround()
        borisLegs.getLegsInMovement()
        borisLegs.getlegsCollided()


if __name__ == '__main__':
  listener()
  
