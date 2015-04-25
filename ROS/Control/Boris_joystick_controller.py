#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import pygame
import math
from math import degrees

pygame.init()
def JoySticktalker():
    pub = rospy.Publisher('servo', String, queue_size=10) #This is publishing the desired servo and the angle to the servo topic. This information is then sent to the arduino and the desired servo is sent
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) #stops trailling commands in Subscriber
    
    joysticks = []
    command_code_index={13: "Forward",
                        12: "StrafeRight",
                        14: "Back",
                        11: "StrafeLeft",
                        3 : "Up",
                        2 : "Down",
                        7 : "Stand",
                        6 : "KILL"
                        }

    joysticks.append(pygame.joystick.Joystick(0))
    joysticks[0].init()
    print "Detected COntroller:",joysticks[0].get_name(),"'"
    
    stringToSend = "_"    
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button in command_code_index:
                    stringToSend = command_code_index[event.button]
                    print command_code_index[event.button]
            elif event.type == pygame.JOYAXISMOTION:
                xAxis=joysticks[0].get_axis(0)*100
                yAxis=joysticks[0].get_axis(1)*100
                #print xAxis,"--x--"
                #print yAxis,"--y--"
                xBound = abs(xAxis)
                yBound = abs(yAxis)
                radius = 30
                deg_angle = math.atan2(yAxis,xAxis)*180/math.pi + 180
                if xBound <= radius and yBound <=radius:
                    stringToSend = command_code_index[7]
                else:
                    if deg_angle >=45 and deg_angle <=135:
                        stringToSend = command_code_index[13]#+","+to_string(yAxis)
                    if deg_angle >= 225 and deg_angle <=315:
                        stringToSend = command_code_index[14]+","+to_string(yAxis)                 
                    if deg_angle > 135 and deg_angle <=225:
                        stringToSend = command_code_index[12]+","+to_string(yAxis) 
                    if deg_angle > 315 and deg_angle <=360 or deg_angle >= 0 and deg_angle <45:
                        stringToSend = command_code_index[11]+","+to_string(yAxis) 
            elif event.type == pygame.JOYBUTTONUP:
                stringToSend = "_"
            
        if stringToSend != "_" and stringToSend != "Stand":
            pub.publish(stringToSend)
        rate.sleep()
if __name__ == '__main__':
    try:
        JoySticktalker()
    except rospy.ROSInterruptException:
        pass
