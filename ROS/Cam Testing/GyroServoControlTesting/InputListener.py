import rospy
import time
#from ServoController import ServoController
from std_msgs.msg import String
from std_msgs.msg import Float32



class Listeners:

	def __init__(self): 
		rospy.init_node('listener', anonymous = True)
		self.zRotation = None
		self.xRotation = None
		self.yRotation = None
		self.xValues = []
		self.yValues = []
		self.zValues = []
		self.counter = 0
		self.start = None

	def countIt(self):
		if time.time() - self.start > 1:
			print(len(self.xValues))
			print(self.counter)
			self.counter = 0


	def callback1(self, value):
		if self.counter == 0:
			self.start = time.time()
		self.xRotation = value
		#print(str(self.xRotation) + "= x")
		self.xValues.append(self.xRotation)
		self.counter = self.counter + 1
		self.countIt()

	def callback2(self, value):
		self.yRotation = value
		#print(str(self.yRotation) + "= y")
		self.yValues.append(self.yRotation)

	def callback3(self, value):
		self.zRotation = value
		#print(str(self.zRotation) + "= z")	
		self.zValues.append(self.zRotation)

	def subscribe(self):
		rospy.Subscriber('xRotation', Float32, self.callback1)
		rospy.Subscriber('yRotation', Float32, self.callback2)
		rospy.Subscriber('zRotation', Float32, self.callback3)
		rospy.spin()

	

tester = Listeners()
tester.subscribe()
rospy.loginfo("this is a test string")