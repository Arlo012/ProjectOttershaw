#This is a test listener file (modified slightly for Gyro readings)

import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Float32



class Listeners:

	def __init__(self): 
		rospy.init_node('listener', anonymous = True)
		self.testvalue = None
		self.xRotation = None

	def callback1(self, message):
		self.testvalue = message
		print(str(self.testvalue) + "= x")

	def callback2(self, value):
		self.xRotation = value
		print(str(self.xRotation) + "= y")

	def subscribe(self):
		rospy.Subscriber('xRotation', Float32, self.callback1)
		rospy.Subscriber('yRotation', Float32, self.callback2)
		rospy.spin()

	

tester = Listeners()
tester.subscribe()
rospy.loginfo("this is a test string")
