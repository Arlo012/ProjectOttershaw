import rospy
import math
import time
import numpy as np
from SpiderLeg import SpiderLeg as leg
from SpiderLeg import Servo
from SpiderLeg import Vector3
from ottershaw_masta.msg import Servo as servoMsg

class ServoController:

	def __init__(self):
		#self.servoPublisher = rospy.Publisher('ServoSpeed', ServoArrayMsg, queue_size = 10)
		self.servoMovePublisher = rospy.Publisher('ServoMove', servoMsg, queue_size = 10)
		rospy.init_node('ServoController', anonymous = True)  #were we to import this as a class to a main IO handler file, this line might cause issues, in which case it should work to simply remove it

	def SendMessage(self, moveArray, servoStep):
		if not rospy.is_shutdown():
			for i in range(len(moveArray)):
				msg = servoMsg()
				msg.ID = i
				msg.angle = moveArray[i]
				msg.stepSize = servoStep[i]
				self.servoMovePublisher.publish(msg)
				time.sleep(0.005)
				

if __name__ == '__main__':
	controller = ServoController()	
	legs = []
	for i in range(0,8):
		servoSetup1 = Servo("Hip",90,90)
		servoSetup2 = Servo("Shoulder",90,90)
		servoSetup3 = Servo("Knee",90,90)
		legToBuild = leg(i, servoSetup1, servoSetup2, servoSetup3)
		legs.append(legToBuild)
		
	while True:
		try:
			#x/y/z coordinates relative to leg to move
			#pointToMove = Vector3(15,10,15) 	#+x = outward, +y = forward, +z = dow
			
			moveServoMessages = []
			stepServoMessage = []
			for leg in legs:
				
				#Create movement array message
				anglesToSend = Vector3(90,90,90)	#Default angle
				#TODO fixme
				#anglesToSend = leg.GetAngles(Vector3(5,0,28))
				#print str(anglesToSend.x) + " " + str(anglesToSend.y) + " " + str(anglesToSend.z)

				#Append 3 move orders
				#moveServoMessages.append(int(anglesToSend.x))
				#moveServoMessages.append(int(anglesToSend.y))
				#moveServoMessages.append(int(anglesToSend.z))
				
				#Append 3 step size orders
				stepServoMessage.append(1)
				stepServoMessage.append(1)
				stepServoMessage.append(1)
				
			#moveServoMessages = [90,90,72,  100,75,75,  80,85,80,  90,90,90,  90,80,90,  90,90,90,  90,90,90,  90,90,90]
			
			#These are the calibrated offsets for 90 90 90
			#moveServoMessages = [92,90,72,  100,72,79,  85,85,85,  86,90,83,  81,80,100,  88,95,78,  89,85,65,  100,81,80]
			moveServoMessages = [92,90,72,  100,72,79,  85,85,85,  86,90,83,  81,80,100,  88,95,78,  89,85,65,  100,81,80]
			stand = [0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0]
			moveServoMessages =  list(np.array(moveServoMessages) + np.array(stand))

			
			#Send over ROS
			controller.SendMessage(moveServoMessages, stepServoMessage)
			#time.sleep(1)
			
		except rospy.ROSInterruptException:
			pass