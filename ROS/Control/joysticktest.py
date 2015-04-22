import time
import rospy
from std_msgs.msg import String
import pygame

from time import sleep



pygame.init()
 
 
def JoyTalker(): 
    
    
    xAxis=0
    yAxis=0
    lastUpdateTime = time.time()
    joysticks = []
    clock = pygame.time.Clock()
    keepPlaying = True
    x={13: "Forward",
       12: "StrafeRight",
       14: "Back",
       11: "StrafeLeft",
       3 : "Up",
       2 : "Down",
       7: "Stand"}
    '''
    full    forward -32767
            back    32767
            right   32767
            left    -32767
    half    forward -16384
            back    16384
            right   16384
            left    -16384
    Quarter forward -8192
            back    8192
            right   8192
            left    -8192   
    '''
    # for al the connected joysticks
    for i in range(0, pygame.joystick.get_count()):
        # create an Joystick object in our list
        joysticks.append(pygame.joystick.Joystick(i))
        # initialize them all (-1 means loop forever)
        joysticks[-1].init()
        # print a statement telling what the name of the controller is
        print "Detected joystick '",joysticks[-1].get_name(),"'"
    while keepPlaying:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print "Received event 'Quit', exiting."
                keepPlaying = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                print "Escape key pressed, exiting."
                keepPlaying = False
            elif event.type == pygame.JOYBUTTONDOWN:
                #print x[event.button]
                print "Joystick '",joysticks[event.joy].get_name(),"' button",event.button,"down."
            elif event.type == pygame.JOYBUTTONUP:
                print "_"
                #print event.button, "up"
                print "Joystick '",joysticks[event.joy].get_name(),"' button",event.button,"up."
            elif event.type == pygame.JOYHATMOTION:
                print "Joystick '",joysticks[event.joy].get_name(),"' hat",event.hat," moved."
            elif event.type == pygame.JOYAXISMOTION:
                #print "Joystick '",joysticks[event.joy].get_name(),"' axis",event.axis,"motion."
                if event.axis==0 or event.axis==1:
                    xAxis=joysticks[0].get_axis(0)
                    print xAxis,"--x--"
                    yAxis=joysticks[0].get_axis(1)
                    print yAxis,"--y--"
                    if xAxis > -8192 and xAxis < 8192 and yAxis > -8192 and yAxis > 8192:# stand
                        print x[7]
                    elif xAxis > -5000 and xAxis < 5000 and yAxis > 8192 and yAxis > 16384:
                        pass
                #if event.axis==1:
                #    yAxis=joysticks[0].get_axis(1)
                #    print yAxis,"--y--"         
 
JoyTalker()
pygame.quit()