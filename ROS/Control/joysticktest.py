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
        pygame.event.pump
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                print "Received event 'Quit', exiting."
                keepPlaying = False
            elif e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE:
                print "Escape key pressed, exiting."
                keepPlaying = False
            elif e.type == pygame.JOYBUTTONDOWN:
                #print x[event.button]
                print "Joystick '",joysticks[event.joy].get_name(),"' button",event.button,"down."
            elif e.type == pygame.JOYBUTTONUP:
                print "_"
                #print event.button, "up"
                print "Joystick '",joysticks[event.joy].get_name(),"' button",event.button,"up."
            elif e.type == pygame.JOYHATMOTION:
                print "Joystick '",joysticks[event.joy].get_name(),"' hat",event.hat," moved."
            elif e.type == pygame.JOYAXISMOTION:
                #print "Joystick '",joysticks[event.joy].get_name(),"' axis",event.axis,"motion."
                if e.axis==0 or e.axis==1:
                    xAxis=joysticks[0].get_axis(0)*100
                    yAxis=-(joysticks[0].get_axis(1)*100)
                    print xAxis,"--x--"
                    print yAxis,"--y--"
                    if xAxis > -28.0 and xAxis < 23.0 and yAxis <= 16.5 and yAxis > -24.0:# stand
                         print x[7]
                    if xAxis > -28 and xAxis < 23 and yAxis <= 78.0 and yAxis > 16.5:#forward
                        print x[13]
                    if xAxis > -28.0 and xAxis < 23.0 and yAxis <= -4.0 and yAxis >= -76.0:#back
                        print x[14]
                    if xAxis > -72.0 and xAxis < -23.0 and yAxis <= 16.6 and yAxis >= -24.0: #left
                        print x[11]
                    if xAxis > 23.0  and xAxis < 72.0 and yAxis <= 16.6 and yAxis >= -24.0: #right
                        print x[12]
                    if xAxis > 32767  or xAxis < -32767 or yAxis > 32767 or yAxis < -32767:
                        print "speed boost!  COMING SOON!!!"
                    '''
                    if xAxis > -28 and xAxis < 23 and yAxis < 16.5 and yAxis > -4:# stand
                         print x[7]
                    if xAxis > -28 and xAxis < 23 and yAxis < 78 and yAxis > 16.5:#forward
                        print x[13]
                    if xAxis > -28 and xAxis < 23 and yAxis < -4 and yAxis > -76:#back
                        print x[14]
                    if xAxis > -72 and xAxis < -23 and yAxis < 16.6 and yAxis > -4: #left
                        print x[11]
                    if xAxis > 23  and xAxis < 72 and yAxis < 16.6 and yAxis > -4: #right
                        print x[12]
                    if xAxis > 32767  or xAxis < -32767 or yAxis > 32767 or yAxis < -32767:
                        print "speed boost!  COMING SOON!!!"
                    '''
                #if event.axis==1:
                #    yAxis=joysticks[0].get_axis(1)
                #    print yAxis,"--y--"         
 
JoyTalker()
pygame.quit()