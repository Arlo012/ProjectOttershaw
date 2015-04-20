#!/usr/bin/env python

import rospy
from ottershaw.msg import Gyro
#file to class in python, python init, create instance of class in some other function, us it to call init, one function in there called moveservo, 


def talker():
    pub = rospy.Publisher('gyro', Gyro, queue_size=10) #This is publishing the desired servo and the angle to the servo topic. This information is then sent to the arduino and the desired servo is sent
    rospy.init_node('gyroTalker', anonymous=True)
    rate = rospy.Rate(10) #Frequency of 10hz

    while not rospy.is_shutdown():
        # Do algorithms here
        
        #Task 1

        pub.publish(Gyro(1,2,3))
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
