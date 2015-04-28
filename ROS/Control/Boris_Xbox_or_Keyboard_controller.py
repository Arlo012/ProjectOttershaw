#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
#file to class in python, python init, create instance of class in some other function, us it to call init, one function in there called moveservo, 
import curses
import pygame
import math
from jinja2.runtime import to_string
from __builtin__ import True

global flag 
flag =False

def KeyboardController():
    stdscr = curses.initscr()           #determines the terminal type and initialises all implementation data structures. The environment variable specifies the terminal type. The initscr() function also causes the first refresh operation to clear the screen.
    curses.cbreak()                     #disables line buffering and erase/kill character-processing (interrupt and flow control characters are unaffected), making characters typed by the user immediately available to the program
    stdscr.keypad(1)                    #The keypad option enables the keypad of the user's terminal. If enabled, the user can press a function key (such as an arrow key) and wgetch returns a single value representing the function key, as in KEY_LEFT.
    
    stdscr.addstr(0,10,"USE 'ctrl + c' TO QUIT,  Control with arrow keys")
    stdscr.refresh()                    #explicitly telling curses to redraw a a terminal with the previous values on screen 

    key = ''
    
    pub = rospy.Publisher('control', String, queue_size=10) #This is publishing the desired servo and the angle to the servo topic. This information is then sent to the arduino and the desired servo is sent
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) #stops trailling commands in Subscriber

    while not rospy.is_shutdown():
        # Do algorithms here
        key = stdscr.getch()            #gets the entered key value and stores it in the "key" variable
        stdscr.addch(1,10,key)          #moves type curser doun 1 line and right 10 spaces
        stdscr.refresh()                # refreshes terminal screen with the above values
        if key == ord('w'):#curses.KEY_UP:  # ord() compares the key value to the character in the ord() function
            #stdscr.addstr(2, 20, "Up")
            stringToSend = "Forward"
        elif key == ord('s'):
            stringToSend = "Back"
        elif key == ord('a'):  
            stringToSend = "StrafeLeft"
        elif key == ord('d'):
            stringToSend = "StrafeRight"
        elif key == ord('z'):
            stringToSend = "Down"
        elif key == ord('x'):
            stringToSend = "Up"
        elif key == ord('p'):
            stringToSend = "Stand"
        elif key == ord('q'):
            stringToSend = "SpinLeft"
        elif key == ord('e'):
            stringToSend = "SpinRight"
        elif key == ord('f'):
            stringToSend = "Freeze"
        #Task 1
        else:
            stringToSend = "STANDBY........"
        pub.publish(stringToSend)
        
        #Task 2
        #Service -- TBD. Need to first merge catkin workspace to git directory
        
        #Task 3
        rate.sleep()
    curses.endwin()         #closes the screen
    

'''
    Sends over ROS control topic 'direction
'''
def JoySticktalker():
    pygame.init()
    pub = rospy.Publisher('control', String, queue_size=10) #This is publishing the desired servo and the angle to the servo topic. This information is then sent to the arduino and the desired servo is sent
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(30) #stops trailing commands in Subscriber
    
    joysticks = []
    B_or_J = raw_input("Brian or Jeff's PC [b/j]\n")
    if B_or_J == 'b':
        command_code_index ={14: "Back",
                            13: "Forward",
                            12: "StrafeRight",
                            11: "StrafeLeft",
                            7 : "Stand",
                            6 : "KILL",
                            5 : "SpinRight",
                            4 : "SipnLeft",
                            3 : "Up",
                            2 : "Down"
                            }
    else:
        command_code_index ={16: "Back",
                            15: "Forward",
                            14: "StrafeRight",
                            13: "StrafeLeft",
                            7 : "Stand",
                            6 : "KILL",
                            5 : "SpinRight",
                            4 : "SipnLeft",
                            3 : "Up",
                            2 : "Down"
                            }

    joysticks.append(pygame.joystick.Joystick(0))
    joysticks[0].init()             #initializes the xbox controler 
    print "Detected Controller:",joysticks[0].get_name(),"'"
    
    stringToSend = "_"    
    while not rospy.is_shutdown():
        for event in pygame.event.get():                # loop through all the posible events that the xboox controller can trigger
            if event.type == pygame.JOYBUTTONDOWN:      # if the event is a from a Button (x,x,a,b, start, d-pad) being pressed is triggered
                if event.button in command_code_index:  # if the event code matches one of the indexes of your command_code_index
                    stringToSend = command_code_index[event.button]
                    print command_code_index[event.button]
            elif event.type == pygame.JOYAXISMOTION:    # if the event is from the LEFT Joystick
                xAxis=joysticks[0].get_axis(0)*100      # get.axis(0) = get the horizontal coordinates of the LEFT joystick
                yAxis=joysticks[0].get_axis(1)*100      # get.axis(1) = get the vertical coordinates of the LEFT joystick
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
                        stringToSend = "Forward"+","+to_string(yAxis)
                    if deg_angle >= 225 and deg_angle <=315:
                        stringToSend = "Back"+","+to_string(yAxis)                 
                    if deg_angle > 135 and deg_angle <=225:
                        stringToSend = "StrafeRight"+","+to_string(xAxis) 
                    if deg_angle > 315 and deg_angle <=360 or deg_angle >= 0 and deg_angle <45:
                        stringToSend = "StrafeLeft"+","+to_string(xAxis) 
            elif event.type == pygame.JOYBUTTONUP:     #if the event is a from a Button (x,x,a,b, start, d-pad) being pressed
                stringToSend = "_"
            
        if stringToSend != "_" and stringToSend != "Stand":
            pub.publish(stringToSend)
        rate.sleep()
if __name__ == '__main__':
    choice = raw_input("Controller or Keyboard [c/k]\n")
    try:
        if choice == 'k':
            KeyboardController()
        elif choice == 'c':
            JoySticktalker()
        else:
            print 'Invalid selection. C for controller, K for keyboard'
    except rospy.ROSInterruptException:
        print '[ERROR] ROS interrupt exception'
