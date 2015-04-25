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
    full -y   forward -32767 -1
         +y   back    32767  .99
         +x   right   32767   .99
         -x   left    -32767 -1
    half    forward -16384   -.5
            back    16384   .495
            right   16384    .495
            left    -16384   -5
    Quarter forward -8192    -.25 
            back    8192     .24
            right   8192    .24
            left    -8192   -.25
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
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                print "Received event 'Quit', exiting."
                keepPlaying = False
            elif e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE:
                print "Escape key pressed, exiting."
                keepPlaying = False
            elif e.type == pygame.JOYBUTTONDOWN:
                #print x[event.button]
                print "Joystick '",joysticks[e.joy].get_name(),"' button",e.button,"down."
            elif e.type == pygame.JOYBUTTONUP:
                print "_"
                #print event.button, "up"
                print "Joystick '",joysticks[e.joy].get_name(),"' button",e.button,"up."
            elif e.type == pygame.JOYHATMOTION:
                print "Joystick '",joysticks[e.joy].get_name(),"' hat",e.hat," moved."
            elif e.type == pygame.JOYAXISMOTION:
                #print "Joystick '",joysticks[event.joy].get_name(),"' axis",event.axis,"motion."
                if e.axis==0 or e.axis==1:
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
                            stringToSend = command_code_index[13]+","+to_string(yAxis)
                        if deg_angle >= 225 and deg_angle <=315:
                            stringToSend = command_code_index[14]+","+to_string(yAxis)                 
                        if deg_angle > 135 and deg_angle <=225:
                            stringToSend = command_code_index[12]+","+to_string(yAxis) 
                        if deg_angle > 315 and deg_angle <=360 or deg_angle >= 0 and deg_angle <45:
                            stringToSend = command_code_index[11]+","+to_string(yAxis) 
                #if event.axis==1:
                #    yAxis=joysticks[0].get_axis(1)
                #    print yAxis,"--y--"         
 
JoyTalker()
pygame.quit()