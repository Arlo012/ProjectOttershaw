#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import pygame


pygame.init()
def JoySticktalker():
    pub = rospy.Publisher('servo', String, queue_size=10) #This is publishing the desired servo and the angle to the servo topic. This information is then sent to the arduino and the desired servo is sent
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) #stops trailling commands in Subscriber
    joysticks = []
    keepPlaying = True
    command_code_index={13: "Forward",
                        12: "StrafeRight",
                        14: "Back",
                        11: "StrafeLeft",
                        3 : "Up",
                        2 : "Down",
                        7 : "Stand"}

    joysticks.append(pygame.joystick.Joystick(0))
    joysticks[0].init()
    print "Detected COntroller '",joysticks[0].get_name(),"'"
    
    stringToSend = "_"    
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print "Received event 'Quit', exiting."
                keepPlaying = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                print "Escape key pressed, exiting."
                keepPlaying = False
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button in command_code_index:
                    stringToSend = command_code_index[event.button]
            elif event.type == pygame.JOYBUTTONUP:
                stringToSend = "_"
        if stringToSend != "_":
            pub.publish(stringToSend)
        rate.sleep()
if __name__ == '__main__':
    try:
        JoySticktalker()
    except rospy.ROSInterruptException:
        pass
