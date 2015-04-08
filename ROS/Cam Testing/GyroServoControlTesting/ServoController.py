import rospy
import math
import time
import numpy as np
from LegMover import *
from SpiderLeg import SpiderLeg as leg
from SpiderLeg import Servo
from SpiderLeg import Vector3
from SpiderLeg import MoveCommand
from ottershaw_masta.msg import Servo as servoMsg

class ServoController:

	def __init__(self):
		#self.servoPublisher = rospy.Publisher('ServoSpeed', ServoArrayMsg, queue_size = 10)
		self.servoMovePublisher = rospy.Publisher('ServoMove', servoMsg, queue_size = 10)
		rospy.init_node('ServoController', anonymous = True)  #were we to import this as a class to a main IO handler file, this line might cause issues, in which case it should work to simply remove it

	def SendMessage(self, moveArray, servoStep):
		if not rospy.is_shutdown():
			for key in moveArray:
				for i in range(0,3):
					msg = servoMsg()			#Create servo message
					msg.ID = (key-1)*3+i		#Convert leg ID to servo ID
					
					#Select x/y/z
					if i is 0:
						msg.angle = moveArray[key].x
						msg.stepSize = servoStep[key].x
					elif i is 1:
						msg.angle = moveArray[key].y
						msg.stepSize = servoStep[key].y
					elif i is 2:
						msg.angle = moveArray[key].z
						msg.stepSize = servoStep[key].z
					else:
						print 'wtf'
					
					self.servoMovePublisher.publish(msg)
					time.sleep(0.003)			#Must delay between each write
			time.sleep(0.005)

if __name__ == '__main__':
	controller = ServoController()	
	legs = []				#Create an array of leg objects (see SpiderLeg)

	#Initialize legs at 90, 90, 90
	for i in range(0,8):
		servoSetup1 = Servo("Hip",90)
		servoSetup2 = Servo("Shoulder",90)
		servoSetup3 = Servo("Knee",90)
		legToBuild = leg(i, servoSetup1, servoSetup2, servoSetup3)
		legs.append(legToBuild)

	'''
	Setup all step commands, labeled
	x/y/z coordinates relative to leg to move: +x = outward, +y = forward, +z = down
	'''
	#Create object to handle leg walking algorithm
	legMover = LegMover(legs)

	while True:
		try:
			#A set of commands to be executed in parallel
			commandSet = legMover.GetNextCommandSet()	#Get next array of move commands
			for command in commandSet:
				moveServoMessages = {}	#Dictionary of angles to send over to Arduino mapped by leg ID
				stepServoMessage = {}	#Dictionary of step sizes for Arduino servo mapped by leg ID
				
				for legID in command.coordinatesToMove:	#Iterate through each command
					
					#print legID
					
					#Update leg positions by mapping
					legs[legID-1].UpdateDesiredPosition(command.coordinatesToMove[legID])
					
					#Calculate angles for these 3 servos 
					anglesToSend = legs[legID-1].GetAngles()
					
					#Update each servo with its associated offset
					anglesToSend.x = anglesToSend.x + calibratedOffsets[legID].x
					anglesToSend.y = anglesToSend.y + calibratedOffsets[legID].y
					anglesToSend.z = anglesToSend.z + calibratedOffsets[legID].z
						
					#Debug
					#anglesToSend.Print()
						
					#Update servo objects to hold these angles accounting for calibration offset
					legs[legID-1].servo1.angle = anglesToSend.x
					legs[legID-1].servo2.angle = anglesToSend.y
					legs[legID-1].servo3.angle = anglesToSend.z
					
					#Append 3 angles
					moveServoMessages[legID] = Vector3(anglesToSend.x, anglesToSend.y, anglesToSend.z)
					
					#Append 3 step size orders
					stepServoMessage[legID] = Vector3(1, 1, 1)
				
					#Send over ROS
					#print 'sent command for ' + str(legID)
					controller.SendMessage(moveServoMessages, stepServoMessage)
			
			#time.sleep(command.timeToExecute/1000)
			time.sleep(.2)

			
		except rospy.ROSInterruptException:
			pass