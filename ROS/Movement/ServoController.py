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
import os

class ServoController:
	def __init__(self):
		rospy.init_node('ServoController', anonymous = True)
		
		#Servo angles and step size messages, publisher
		self.servoMovePublisher = rospy.Publisher('ServoMove', servoMsg, queue_size = 10)
		
		#Setup listener for remote control
		rospy.Subscriber('control', String, self.ReceiveCommand)
		
		#Setup debug channel for publishing
		self.debugChannel = rospy.Publisher('Debug', String)
		time.sleep(0.3)
		self.debugChannel.publish('[INFO] Initialized remote controller ROS listener')

	'''
	Process keyboard commands sent over ROS. 
	Requires the legMover object to be initialized
	'''
	def ReceiveCommand(self, direction):
		try:
			self.legMover.SetMovementCommand(direction.data)
		except Exception as e:
			self.debugChannel.publish('[ERROR] Command not received: ' + str(e))
		
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
	print 'Starting main ServoController process...'
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
					anglesToSend = legs[legID-1].GetAngles()		#If out of bounds will throw error
					
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
			
					
			#Delay between each command - pull from the command's timeToExecute field (set by LegMover command generation)
			delayTime = (float)(command.timeToExecute)/1000.0
			time.sleep(delayTime)
	
		except Exception as e:
			#Grab debug information about line number (see: http://stackoverflow.com/questions/1278705/python-when-i-catch-an-exception-how-do-i-get-the-type-file-and-line-number)
			exc_type, exc_obj, exc_tb = sys.exc_info()
			fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
			lineError = (exc_type, fname, exc_tb.tb_lineno)
			controller.debugChannel.publish('[ERROR] Main command loop failure. ' + str(lineError))
			
			#Recover with stand command
			commandSet = legMover.GetStandCommand()
			for command in commandSet:
				moveServoMessages = {}	#Dictionary of angles to send over to Arduino mapped by leg ID
				stepServoMessage = {}	#Dictionary of step sizes for Arduino servo mapped by leg ID
				
				for legID in command.coordinatesToMove:		#Iterate through each command
					#Update leg positions by mapping
					legs[legID-1].UpdateDesiredPosition(command.coordinatesToMove[legID])																
					
					#Calculate angles for these 3 servos 
					anglesToSend = legs[legID-1].GetAngles()		#If out of bounds will throw error
					
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
			controller.debugChannel.publish('[WARNING] Recovered by returning to standing position')
			
	controller.debugChannel.publish('[INFO] Servo controller shutdown command received. Exiting...')
			