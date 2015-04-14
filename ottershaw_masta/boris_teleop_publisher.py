#!/usr/bin/env python
import rospy
from std_msgs.msg import String
#file to class in python, python init, create instance of class in some other function, us it to call init, one function in there called moveservo, 
import curses
stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)

stdscr.addstr(0,10,"USE 'ctrl + c' TO QUIT,  Control with arrow keys")
stdscr.refresh()

key = ''

def talker():
    pub = rospy.Publisher('servo', String, queue_size=10) #This is publishing the desired servo and the angle to the servo topic. This information is then sent to the arduino and the desired servo is sent
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
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
