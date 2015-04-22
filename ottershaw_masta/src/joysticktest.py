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
                print x[event.button]
                #print "Joystick '",joysticks[event.joy].get_name(),"' button",event.button,"down."
            elif event.type == pygame.JOYBUTTONUP:
                print "_"
                #print event.button, "up"
                #print "Joystick '",joysticks[event.joy].get_name(),"' button",event.button,"up."
        
        '''
            elif event.type == pygame.JOYHATMOTION:
                print "Joystick '",joysticks[event.joy].get_name(),"' hat",event.hat," moved."
            elif event.type == pygame.JOYAXISMOTION:
                #print "Joystick '",joysticks[event.joy].get_name(),"' axis",event.axis,"motion."
                if event.axis==0:
                    xAxis=joysticks[-1].get_axis(0)
                if event.axis==1:
                    pass
                   # yAxis=joysticks[-1].get_axis(1)        
        '''  
 
JoyTalker()
pygame.quit()