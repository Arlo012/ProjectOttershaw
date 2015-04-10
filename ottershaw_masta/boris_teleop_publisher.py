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
        elif key == ord('s'):#curses.KEY_DOWN: 
            #stdscr.addstr(3, 20, "Down")
            stringToSend = "Back"
        elif key == ord('a'):#curses.KEY_LEFT:     
            #stdscr.addstr(3, 20, "Down")
            stringToSend = "StrafeLeft"
        elif key == ord('d'):#curses.KEY_RIGHT: 
            #stdscr.addstr(3, 20, "Down")
            stringToSend = "StrafeRight"
        elif key == ord('z'):#curses.KEY_RIGHT: 
            #stdscr.addstr(3, 20, "Down")
            stringToSend = "Down"
        elif key == ord('x'):#curses.KEY_RIGHT: 
            #stdscr.addstr(3, 20,UP "Down")
            stringToSend = "Up"
        elif key == ord('p'):#curses.KEY_RIGHT: 
            #stdscr.addstr(3, 20, "Down")
            stringToSend = "Stand"
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
