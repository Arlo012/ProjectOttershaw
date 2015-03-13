#!/usr/bin/env python

import rospy
import os
from std_msgs.msg import Int32



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
  
