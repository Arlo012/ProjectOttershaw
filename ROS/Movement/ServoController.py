#!/usr/bin/python
import rospy
import math
import time
import numpy as np
import sys
from std_msgs.msg import String
from LegMover import *
from SpiderLeg import SpiderLeg as leg
from SpiderLeg import Servo
from SpiderLeg import Vector3
from SpiderLeg import MoveCommand
from ottershaw_masta.msg import Servo as servoMsg

class ServoController:
	
	#Defined commands in 'boris_teleop_publisher' that we expect to receive over ROS 
	AllowedCommands = ['Forward', 'Back', 'StrafeLeft', 'StrafeRight', 
						'Up', 'Down', 'Stand', 'SpinRight', 'SpinLeft', 'Freeze']

	def __init__(self):
		#Servo angles and step size messages, publisher
		self.servoMovePublisher = rospy.Publisher('ServoMove', servoMsg, queue_size = 10)
		
		#Setup listener for remote control
		rospy.Subscriber('servo', String, self.ReceiveCommand)
		print '[INFO] Initialized remote controller ROS listener'
		
		#Setup debug channel for publishing
		self.debugChannel = rospy.Publisher('Debug', String)
		
		rospy.init_node('ServoController', anonymous = True)

	'''
	Process keyboard commands sent over ROS. 
	Requires the legMover object to be initialized
	'''
	def ReceiveCommand(self, direction):
		try:
			if direction.data in ServoController.AllowedCommands:
				self.legMover.SetMovementCommand(direction.data)
			else:
				print '[WARNING] Issued unrecognized command. Could not send leg controller'
		except:
			print "[ERROR] Unexpected error while generating movement command:", sys.exc_info()[0]

	'''
	Get a pointer to the debug channel to pass around to other objects
	for debugging
	'''
	def GetDebugChannel(self):
		return self.debugChannel
	
	'''
	Define instance of LegMover object for remote controlling
	'''
	def SetLegMoverObject(self, legMover):
		self.legMover = legMover
	
	'''
	Create and send a servo move message and send over ROS
	'''
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
						self.debugChannel.publish('wtf')
					
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

	#Create object to handle leg walking algorithm. This is needed for the controller
	controller.debugChannel.publish('[INFO] Initializing leg controller object...')
	legMover = LegMover(legs, controller.GetDebugChannel(), 'Standard', calibrationMode=False)
	controller.SetLegMoverObject(legMover)
	controller.debugChannel.publish('[INFO] ...Leg controller successfully initialized')

	'''
	Main loop: While roscore running:
		1. Get a set of commands from the legMover class
		2. Process each command in that command set
			- The command will have a mapping of legID : coordinate
			- Go through each of these mappings, generate angles, and append them to the messages to send
		3. Send the messages for that command set over ROS
		4. Delay before processing next command set
	'''
	while True:
		try:
			#A set of commands to be executed in parallel
			commandSet = legMover.GetNextCommandSet()	#Get next list of move commands
			for command in commandSet:
				moveServoMessages = {}	#Dictionary of angles to send over to Arduino mapped by leg ID
				stepServoMessage = {}	#Dictionary of step sizes for Arduino servo mapped by leg ID
				
				for legID in command.coordinatesToMove:		#Iterate through each command
					#Update leg positions by mapping
					legs[legID-1].UpdateDesiredPosition(command.coordinatesToMove[legID])																
					
					#Calculate angles for these 3 servos 
					try:
						anglesToSend = legs[legID-1].GetAngles()
					except Exception as e:										
						controller.debugChannel.publish('[ERROR] Error calculating angles: ' + str(e))
					
					#Update each servo with its associated offset
					anglesToSend.x = anglesToSend.x + calibratedOffsets[legID].x
					anglesToSend.y = anglesToSend.y + calibratedOffsets[legID].y
					anglesToSend.z = anglesToSend.z + calibratedOffsets[legID].z
						
					#Update servo objects to hold these angles accounting for calibration offset
					legs[legID-1].servo1.angle = anglesToSend.x
					legs[legID-1].servo2.angle = anglesToSend.y
					legs[legID-1].servo3.angle = anglesToSend.z
					
					#Append 3 angles
					moveServoMessages[legID] = Vector3(anglesToSend.x, anglesToSend.y, anglesToSend.z)
					
					#Append 3 step size orders
					stepServoMessage[legID] = Vector3(1,1,1)
				
					#Send over ROS
					controller.SendMessage(moveServoMessages, stepServoMessage)
			
			#Delay between each command
			#time.sleep(command.timeToExecute/1000)
			time.sleep(.15)

			
		except rospy.ROSInterruptException:
			pass
		