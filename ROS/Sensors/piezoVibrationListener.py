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
#  listener()

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRI  listener()
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE  listener()

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
from std_msgs.msg import String 
import graphics as gpy

class SpiderLegs:
    '''  listener()

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
    def getLegsOnGround(self):
        print legsOnGround
        print "-----------"

    def getLegsInMovement(self):
        print legsInMovement
        print "-----------"

    def getlegsCollided(self):
        print legsCollided
        print "----------"

borisLegs = SpiderLegs()
centers = []
#canvas = gpy.GraphWin('Leg Step Detection', 1000,1000)
legStatusIndicators = {}
#Create dictionary of leg number to value: initialize values at 0
#Uncomment the appropriate line based on your microcontroller board
#legValues = {54:0, 55:0, 56:0, 57:0, 58:0, 59:0, 60:0, 61:0} 
#legs = [54, 55, 56, 57, 58, 59, 60, 61]
legValues = {14:0, 15:0, 16:0, 17:0, 18:0, 19:0}

stepThreshold = 40   #threshold for leg step completion
noiseThreshold = 30 #noise from other legs moving, servo jitter, etc.

def listener(): 
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that muingltiple talkers can
    # run simultaneously.
    rospy.init_node('LegPiezo', anonymous=True)

    rospy.Subscriber('piezo', String, piezoCall)
    
    rospy.spin()
    dataFile.close()


def piezoCall(data):
    '''
    Do work for this particular call using 'data'  listener()

    This function is called every time a new data comes in on the listener.

    Inputs:
        data = int of values for each leg

    Value determines if the leg's on the ground (0), in the air (1), or collided (-1)
    '''

   #TODO: Test if leg in movement generates enough vibration to be detected by sensor
   #to use inverted values, i.e. using 0 for completed step and 1 for leg in movement

    #Simple work-around for multi-threading problem
    #write data into file
    '''
    Temporary work around Tkinter is not multi-thread safe. 
    Will write file and use other script to read until 
    implementation of mtTkinter, a multi-thread safe wrapper class.
    '''
    dataFile = open('ottershaw_piezoData.txt', 'w')
    dataFile.truncate()
    dataFile.write('%s'%data.data + '\n')
    # dataFile.close()

    '''
    Uncomment rest of code once mtTkinter is implemented
    '''
    # piezoReadings = data.data.split()   #List of 12 or 16 items with leg number followed by value

    # for i, data in enumerate(piezoReadings):
    #         legValues[14+i] = data

    # CategorizeLegStatus()
    

def CategorizeLegStatus():
    '''
    Separate legs pins depending on their status.
    Will be used for feedback control together with gyroscopic sensor
    '''
    for leg in legValues.keys():
        if legValues[leg] < stepThreshold and legValues[leg] > noiseThreshold:
            borisLegs.legsInMovement.append(leg)
        elif legValues[leg] > stepThreshold:     
            borisLegs.legsCollided.append(leg)
        elif legValues[leg] < noiseThreshold:    
            borisLegs.legsOnGround.append(leg)
        else:
            print "Invalid Status on leg %s" % leg

def DrawCircles():
    #xrange 1,4 for 6 circles and just 4 for 8 circles
    for i in xrange(1, 4):
        centers.append(gpy.Point(200, 200*i))
        centers.append(gpy.Point(500, 200*i))

    #Create center points for circle indicators
    for i, point in enumerate(centers):
        #Uncomment the appropriate line based on your microcontroller board
        #legStatusIndicators[54+i] = gpy.Circles(point, 5)
        legStatusIndicators[14+i] = gpy.Circle(point, 50)
        
    #Draw circles with red fill
    for indicator in legStatusIndicators.keys():
        legStatusIndicators[indicator].setOutline('red')
        legStatusIndicators[indicator].setFill('red')
        legStatusIndicators[indicator].draw(canvas)

def UpdateIndicatorStatus():
    for leg in legValues.keys():
            for indicator in legStatusIndicators.keys():
                if indicator == leg:
                    if legValues[leg] > noiseThreshold and legValues[leg] < stepThreshold:
                        legStatusIndicators[indicator].setOutline('red')
                        legStatusIndicators[indicator].setFill('red')
                    if legValues[leg] > stepThreshold:
                        legStatusIndicators[indicator].setOutline('green')
                        legStatusIndicators[indicator].setFill('green')
                    if legValues[leg] < noiseThreshold:
                        legStatusIndicators[indicator].setOutline('blue')
                        legStatusIndicators[indicator].setFill('blue')

if __name__ == '__main__':
    listener()



    