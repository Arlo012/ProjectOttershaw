#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
#file to class in python, python init, create instance of class in some other function, us it to call init, one function in there called moveservo, 
import curses
import pygame
from jinja2.runtime import to_string

def KeyboardController():
    stdscr = curses.initscr()           #TODO document me
    curses.cbreak()                     #all of me....
    stdscr.keypad(1)
    
    stdscr.addstr(0,10,"USE 'ctrl + c' TO QUIT,  Control with arrow keys")
    stdscr.refresh()

    key = ''
    
    pub = rospy.Publisher('control', String, queue_size=10) #This is publishing the desired servo and the angle to the servo topic. This information is then sent to the arduino and the desired servo is sent
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100) #stops trailling commands in Subscriber

    while not rospy.is_shutdown():
        # Do algorithms here
        key = stdscr.getch()
        stdscr.addch(1,10,key)
        stdscr.refresh()
        if key == ord('w'):#curses.KEY_UP: 
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
    curses.endwin()
    

'''
    Sends over ROS control topic 'direction
'''
def JoySticktalker():
    pygame.init()
    pub = rospy.Publisher('control', String, queue_size=10) #This is publishing the desired servo and the angle to the servo topic. This information is then sent to the arduino and the desired servo is sent
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(30) #stops trailing commands in Subscriber
    
    joysticks = []
    command_code_index={14: "Back",
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

    joysticks.append(pygame.joystick.Joystick(0))
    joysticks[0].init()
    print "Detected Controller:",joysticks[0].get_name(),"'"
    
    stringToSend = "_"    
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button in command_code_index:
                    stringToSend = command_code_index[event.button]
                    print command_code_index[event.button]
            elif event.type == pygame.JOYAXISMOTION:
                xAxis=joysticks[0].get_axis(0)*100
                yAxis=-(joysticks[0].get_axis(1)*100)
                #print xAxis,"--x--"
                #print yAxis,"--y--"
                if xAxis >= -30.0 and xAxis <= 15.0 and yAxis <= 3.5 and yAxis >= -11.0:  #Stand (dead zone)
                    stringToSend = command_code_index[7]
                    print xAxis,"--x--"
                    print yAxis,"--y--"
                if xAxis >= -30.0 and xAxis <= 15.0 and yAxis <= 100.0 and yAxis >3.5: #Forward
                    stringToSend = command_code_index[13]+","+to_string(yAxis)
                    print xAxis,"--x--"
                    print yAxis,"--y--"
                if xAxis >= -30.0 and xAxis <= 15.0 and yAxis <= -11.0 and yAxis >= -99.0: #Back
                    stringToSend = command_code_index[14]+","+to_string(yAxis)
                    print xAxis,"--x--"
                    print yAxis,"--y--"
                if xAxis >= -100.0 and xAxis < -30.0 and yAxis <= 3.5 and yAxis >= -11.0: #Left
                    stringToSend = command_code_index[11]+","+to_string(xAxis)
                    print xAxis,"--x--"
                    print yAxis,"--y--"
                if xAxis >= 15.0  and xAxis <= 99.0 and yAxis <= 3.5 and yAxis >= -11.0: #Right
                    stringToSend = command_code_index[12]+","+to_string(xAxis)
                    print xAxis,"--x--"
                    print yAxis,"--y--"
            elif event.type == pygame.JOYBUTTONUP:
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
