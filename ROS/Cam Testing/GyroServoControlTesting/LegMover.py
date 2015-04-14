from SpiderLeg import MoveCommand
from SpiderLeg import Vector3
import copy
from collections import deque
import time

'''
Implements basic forward walking motion on a flat surface
'''

#Starting point from which to base the standing coordinate
baseCoor = Vector3(15, 0, 10)	#90, 90, 90 at (13.4, 0, 21.2)

#Alternating positive/negative standing offset
splayFactor = 5		#How far legs splay apart in standing position

#Note that these standing coordinates are in local leg coordinates, not body coordinates
standingCoordinates = {1: Vector3.Add(baseCoor, Vector3(0,splayFactor,0)),	#Leg1
			2 : Vector3.Add(baseCoor, Vector3(0,-splayFactor,0)),   		#Leg2
			3 : Vector3.Add(baseCoor, Vector3(0,splayFactor,0)),			#Leg3
			4 : Vector3.Add(baseCoor, Vector3(0,-splayFactor,0)),			#Leg4
			5 : Vector3.Add(baseCoor, Vector3(0,splayFactor,0)),			#Leg5
			6 : Vector3.Add(baseCoor, Vector3(0,-splayFactor,0)),			#Leg6
			7 : Vector3.Add(baseCoor, Vector3(0,splayFactor,0)),			#Leg7
			8 : Vector3.Add(baseCoor, Vector3(0,-splayFactor,0))			#Leg8		
}

ninetyCoordinates = {1: Vector3(13,0,21),	#Leg1
			2: Vector3(13,0,21),			#Leg2
			3: Vector3(13,0,21),			#Leg3
			4: Vector3(13,0,21),			#Leg4
			5: Vector3(13,0,21),			#Leg5
			6: Vector3(13,0,21),			#Leg6
			7: Vector3(13,0,21),			#Leg7
			8: Vector3(13,0,21)				#Leg8		
}

#These are the angle calibrated offsets for 90 90 90 (with subtractions for offset to spread)
calibratedOffsets = {1 : Vector3(2,5,-18), 
			2 : Vector3(10,-10,-11),  
			3 : Vector3(-5,-3,-2),
			4 : Vector3(-4,2,-7),  
			5 : Vector3(-9,-3,11),
			6 : Vector3(-2,10,-12),
			7 : Vector3(-1,-8,-25),
			8 : Vector3(10,-12,-15)
}


class Movement:
	'''
	Associates a series of motions (differential vectors) with an 
	amount of time to execute them. 
	
	Note that it is a requirement that each motion in this set take an 
	identical amount of time to execute. Movements with complex timings
	should be separated into different movements, and separated in their 
	associated movement list.
	'''
	def __init__(self, motion, timeToExecute=300, coordinatePlane='Robot'):
		'''
		Args:
			motion ([Vector3]): list of Vector3 objects to execute as part of this movement
			timeToExecute: how much time must be alloted for this movement (ms)
			coordinatePlane: use local leg coordinate plane (+x = out), or robot coordinate plane
				+x = RIGHT of robot
		'''
		self.motion = motion
		self.timeToExecute = timeToExecute
		self.coordinatePlane = coordinatePlane

#Debug & safety modes
debugMode = False					#Explicit printouts of leg positions for debugging

#ENABLE THIS WHEN TESTING A NEW WALKING SET
safeAlgorithmLoopMode = False		#Always return to standard splay position at end of each movement loop

class LegMover:
	'''
	Contains the algorithms for moving spider legs. 
	
	Supports differential movement lists, pre-defined or function-generated arrays
	of differential points from the current position to move the legs. On initialization,
	this object maps predefined sets of legs (moveSets) to these differential movement arrays.
	
	Each command is executed in parallel for each leg in moveSets mapped against its differential
	move array. Once completed, the next leg set : movement array is executed in parallel, etc.
	
	A note on semantics: 
		A MOVEMENT is made up of a series of MOTIONS, e.g. a MOVEMENT up and forward will consist of
		a series of MOTIONS to arrive there
	'''
	
	#Sets of legs to move together, accessed by index in the legMovementDicts
	moveSets = [ 
			[1,3,5,7], [2,4,6,8],	#(0,1)Standard walking sets
			[2,6], [3,7],		  	#(2,3)Sides by leg ID
			[1,5], [4,8], 			#(4,5) Front/back by leg ID	
			[1], [2], [3], [4],		#(6-9)
			[5], [6], [7], [8],		#(10-13)
			[1,2,3,4,5,6,7,8]		#(14)
	]
  
	'''
	Differential movement lists:
		A list of movement vectors to be executed serially by a set of legs defined in moveSet.
		These are differential, meaning that if a leg begins at position (13,0,15) and the vector list
		is of the form [Vector3(0,-2.5,-5), Vector3(0,2.5,5)], the leg will move to (13, -2.5, 10) and then
		back to (13,0,15).
	Form: [Vector3(x,y,z), Vector3(x,y,z)....]	to be executed serially by a moveSet
	'''
	
#Ripple Gait
	#FWD
	liftUpFwdMotion = [Vector3(0,-2.5,-5)]
	putDownFwdMotion = [Vector3(0,-2.5,5)]
	
	#REV
	liftUpReverseMotion = [Vector3(0,2.5,-5)]
	putDownReverseMotion = [Vector3(0,2.5,5)]

#Wave Gait
	#FWD
	standardFwdStepMotion = [Vector3(0,-2.5,-5), Vector3(0,-2.5,5)]
	halfFwdHump = [Vector3(0,5,0)]
	
	#REV
	standardRevStepMotion = [Vector3(0,2.5,-5), Vector3(0,2.5,5)]
	halfRevHump = [Vector3(0,-5,0)]
	
#Set down
	sitDownMotion = [Vector3(0,0,-8)]
	plusXMotion = [Vector3(5,0,-5)]

#Sit Up
	sitUpMotion = [Vector3(0,0,8)]
	minusXMotion = [Vector3(-5,0,5)]

#Do nothing motion (freeze the robot in current location)
	nothingMotion = [Vector3(0,0,0)]

#Strafe left motions
	leftStrafeMotion = [Vector3(-2.5,0,-5), Vector3(-2.5,0,5)]
	halfLeftHump = [Vector3(5,0,0)]
	
#Strafe right motions
# 	strafeRevMotion = [Vector3(2.5,0,-5), Vector3(2.5,0,5)]
# 	halfLeftRevHump = [Vector3(-5,0,0)]	

	
#Test Motions	
	#For stretch test
	stretchMotion = [Vector3(0,-7.5,0)]
	
	#Foot stomping test
	stompUpMotion = [Vector3(0,0,-5)]
	stompDownMotion = [Vector3(0,0,5)]
	
	#Above Z test
	holdUpMotion = [Vector3(0,0,-10)]
	holdDownMotion = [Vector3(0,0,10)]
	
	swingMotion = [Vector3(10,0,0), Vector3(-10,0,0)]	
	
	def __init__(self, legArray, moveMode='Standard', commandBufferSize=3, calibrationMode = False):
		'''
		Args:
			legArray ([SpiderLeg]): array of SpiderLeg objects to act upon
			moveMode: 'Standard' or 'PacMan' -- move one command at a time, or continuously in a direction (respectively)
		'''
		self.spiderLegs = legArray
		self.currentLegCoordinates = copy.deepcopy(standingCoordinates)  #Default coordinates to standing position
		self.legMovementDictToProcess = 0				#Which legMovementDict we are accessing
		self.nextMotionIndex = 0						#Which motion within each differential movement array we are at
		
		#Move 'standard' style (i.e. 'Call of duty', holding forward moves forward) or 'PacMan' style (continue in a line)
		self.moveMode = moveMode		#'Standard' executes each command set only once, and PacMan loops it continuously
		
		#Calibration mode (for 90/90/90'ing the legs). Forces return of Get90DegreeCommand() whenever new coordinates polled
		self.calibrationMode = calibrationMode
		
		
		#Print out all relevant configurations
		print '[CONFIG] Current movement mode is ' + self.moveMode
		if self.calibrationMode:
			print '[CONFIG] Currently in calibration mode (robot will 90/90/90 his legs, regardless of orders)'
		if debugMode:
			print '[CONFIG] LegMover debug mode is initialized! Verbose logging enabled.'
		else:
			print '[CONFIG] LegMover debug mode disabled. Verbose logging will not be provided.'
		if safeAlgorithmLoopMode:
			print '[CONFIG] Algorithm safety loop mode enabled. This will force the robot to return to standing after each motion.'
		
		'''
		Goal: Above we have lists of differential motions. Next, map these motions in a given order against 
		leg sets to be executed!
		
		List of mappings of sets of legs (moveSet) list INDEX to a differential movement arrays. These are executed serially.
		Everything inside one dictionary is executed in parallel.
		
		Example (wave the legs!): 
			#Sets of legs to move together, accessed by index in the legMovementDicts (SEE ABOVE)
			moveSets = [ 
			[1,3,5,7], [2,4,6,8],	#(0,1)Standard walking sets
			[2,6], [3,7],		  	#(2,3)Sides by leg ID
			[1,5], [4,8], 			#(4,5) Front/back by leg ID	
			[1], [2], [3], [4],		#(6-9)
			[5], [6], [7], [8],		#(10-13)
			[1,2,3,4,5,6,7,8]		#(14)
			]
		
			wavingMovements = [ 
				{0 : LegMover.holdUpMotion},		#Move legs 1,3,5,7 up
				{0 : LegMover.moveForwardMotion},	#Move legs 1,3,5,7 forward (first part of wave)
				{0 : LegMover.moveBackMotion},		#Move legs 1,3,5,7 back (second part of wave)
				{0 : LegMover.holdDownMotion},		#Move legs 1,3,5,7 back to ground
				
				#Repeat above for just legs 1 & 8
				{6 : LegMover.holdUpMotion, 13: LegMover.holdUpMotion}, #Move legs 1 & 8 up in parallel
				{6 : LegMover.moveForwardMotion, 13: LegMover.moveForwardMotion},
				{6 : LegMover.moveBackMotion, 13: LegMover.moveBackMotion},
				{6 : LegMover.holdDownMotion, 13: LegMover.holdDownMotion}
			]
		'''
		waveForwardMovements = [ 
			{6 : Movement(LegMover.standardFwdStepMotion)},
			{7 : Movement(LegMover.standardFwdStepMotion)},
			{8 : Movement(LegMover.standardFwdStepMotion)},
			{9 : Movement(LegMover.standardFwdStepMotion)},
			{10 : Movement(LegMover.standardFwdStepMotion)},
			{11 : Movement(LegMover.standardFwdStepMotion)},
			{12 : Movement(LegMover.standardFwdStepMotion)},
			{13 : Movement(LegMover.standardFwdStepMotion)},
			{14 : Movement(LegMover.halfFwdHump, 500)}
		]
		
		waveBackwardMovements = [
			{6 : Movement(LegMover.standardRevStepMotion)},
			{7 : Movement(LegMover.standardRevStepMotion)},
			{8 : Movement(LegMover.standardRevStepMotion)},
			{9 : Movement(LegMover.standardRevStepMotion)},
			{10 : Movement(LegMover.standardRevStepMotion)},
			{11 : Movement(LegMover.standardRevStepMotion)},
			{12 : Movement(LegMover.standardRevStepMotion)},
			{13 : Movement(LegMover.standardRevStepMotion)},
			{14 : Movement(LegMover.halfRevHump, 500)}
		]
		
		rippleForwardMovements = [ 
			#First half
	 		{0 : Movement(LegMover.liftUpFwdMotion, 400)},
	 		{1 : Movement(LegMover.halfFwdHump, 500)},
	 		{0 : Movement(LegMover.putDownFwdMotion, 400)},		
	
	 		#Second half
	 		{1 : Movement(LegMover.liftUpFwdMotion, 400)},
	 		{0 : Movement(LegMover.halfFwdHump, 500)},
	 		{1 : Movement(LegMover.putDownFwdMotion, 400)}
		]
		
		rippleBackwardsMovements = [
	 		{1 : Movement(LegMover.liftUpReverseMotion, 400)},
	 		{0 : Movement(LegMover.halfRevHump, 500)},
	 		{1 : Movement(LegMover.putDownReverseMotion, 400)},
	 		
	 		{0 : Movement(LegMover.liftUpReverseMotion, 400)},
	 		{1 : Movement(LegMover.halfRevHump, 500)},
	 		{0 : Movement(LegMover.putDownReverseMotion, 400)}
		]
		
#WORKING AREA
		strafeLeftMovements = [
			{0: Movement(LegMover.leftStrafeMotion,400)},
			{1: Movement(LegMover.halfLeftHump, 500)},
 			{0: Movement(LegMover.leftStrafeMotion, 400)},
		
 			{1: Movement(LegMover.leftStrafeMotion,400)},
 			{0: Movement(LegMover.halfLeftHump, 500)},
 			{1: Movement(LegMover.leftStrafeMotion, 500)}
		]

		doNothingMovements = [
			{14 : Movement(LegMover.nothingMotion, 200)}		
		]
		
		sitDownMovements = [
			{14 : Movement(LegMover.sitDownMotion, 700)},
			{14 : Movement(LegMover.plusXMotion, 700, 'Leg')}		#Move in local leg coordinates
			#NOT FINISHED
		]
		
		sitUpMovements = [
			{14 : Movement(LegMover.minusXMotion, 700, 'Leg')},		#Move in local leg coordinates
			{14 : Movement(LegMover.sitUpMotion, 700)}
			#NOT FINISHED
		]
		
		plusXMovements = [
			{14 : Movement(LegMover.plusXMotion)}		
		]
#/WORKING AREA			
		'''
		Map above movements against a given command to the robot. These will be executed per commands
		from the ServoController, sent in via SetMovementCommand(command) below.
		'''
		self.commandLegMovementDict = {
			'Nothing' : doNothingMovements,
			'Forward' : rippleForwardMovements,
			'StrafeLeft' : strafeLeftMovements,
			'Back' : rippleBackwardsMovements,
			'Down' : sitDownMovements,
			'SpinLeft' : plusXMovements,
			'Down' : sitDownMovements,
			'Up' : sitUpMovements,
			'Freeze' : doNothingMovements
		}
		
		#LIFO on all commands (only relevant in standard movement mode)
		self.commandQueue = deque(maxlen=commandBufferSize)
		
		#Default, do nothing. This is the dictionary processed in the main GetNextCommandSet() loop
		self.legMovementDicts = doNothingMovements
# 		self.legMovementDicts = rippleForwardMovements		#Hard code move forward
		
		#Flags for safe walking testing & PacMan mode - Reset to standing after new command flag is set
		self.resetOnceFlag = False
		self.standingMode = False
	
	'''
	Set the current movement command out of the pre-defined movements (see init)
	Command: String of form
		Forward, Back, Left, Right, SpinLeft, SpinRight, Up, Down, Stand
	'''
	def SetMovementCommand(self, command):
		print '[INFO] New movement command: ' + command
		
		#Try finding a match between this command and the dictionary of available commands
		try:
			commandLookup = self.commandLegMovementDict[command]		#Lookup command in commandLegMovementDict
			print '[INFO] Successfully received move command'
			
			#HACK
			if command is 'Freeze':
				self.legMovementDicts = self.commandLegMovementDict['Nothing']
		except:
			print '[WARNING] Invalid movement command sent to robot. Returning to standing'
			self.standingMode = True
			self.legMovementDicts = self.commandLegMovementDict['Nothing']
			return
		
		#Found command, process based on moveMode
		if self.moveMode is 'PacMan':		#Ignore the command queue -- this command is now all we care about
			self.resetOnceFlag = True		#For safety, return to the standing position first
			if command == 'Stand':
				print '[INFO] Stand command received. Robot returning to standing position'
				self.standingMode = True
			else:
				self.standingMode = False
				self.legMovementDicts = commandLookup		#Ignoring queue, over-write current movement command with new one
					
		elif self.moveMode is 'Standard':	#Respect the command queue -- this command is added to the to-be-executed
			print '[INFO] Appended command to queue'
			self.commandQueue.append(commandLookup)	#Append to 'right' of queue, get from 'left'
		
		else:
			print '[ERROR] Tried to set a movement command with uninitialized or invalid movement mode. Returning to standing'
			self.standingMode = True
	
	'''
	Returns a series of commands for the next leg movement
	Use nextMotionIndex value to determine which legs to move,
	indexing into movement array using nextMovementVectorIndex
	'''
	def GetNextCommandSet(self):
		commandSet = []		 #List of MoveCommand objects to return
		
		if debugMode:
			print '[INFO] Current leg coordinates:'
			for leg in self.currentLegCoordinates:
				self.currentLegCoordinates[leg].Print()
			print
		
		#90/90/90 calibration mode
		if self.calibrationMode:
			return self.Get90DegreeCommand()
		
		#Stand up mode
		if self.standingMode:
			print '[INFO] I am standing'
			self.currentLegCoordinates = copy.deepcopy(standingCoordinates)
			return self.GetStandCommand()
		
		#Stand coordinates for transitioning between commands
		elif self.resetOnceFlag:
			print '[INFO] Resetting to standing for new command'
			self.currentLegCoordinates = copy.deepcopy(standingCoordinates)
			self.resetOnceFlag = False
			return self.GetStandCommand()
		
		#Is the movement dictionary fully processed?
		if self.legMovementDictToProcess >= len(self.legMovementDicts):
			self.nextMotionIndex = 0				#Set flag to begin algorithm over again
			self.legMovementDictToProcess = 0		#Set flag to begin at top of the first differential movement array
				
			if self.moveMode is 'PacMan':				#Loop movement continuously
				if safeAlgorithmLoopMode:	#Reset to standing position
					print '[INFO] Reset to standing position'
					self.currentLegCoordinates.clear()
					self.currentLegCoordinates = copy.deepcopy(standingCoordinates)		#Create deep (by-value) copy and replace current dictionary
					return self.GetStandCommand()	#Reset to normal standing position
				else:
					return self.GetNextCommandSet()
			
			elif self.moveMode is 'Standard':
				if self.commandQueue:		#If deque not empty (weird syntax, but easiest way to check)
					#Grab next command from the queue
					self.legMovementDicts = self.commandQueue.popleft()		#Pop from left, put to right (see set commands)
					if debugMode:
						print '[INFO] Got next command in queue, ' + str(len(self.commandQueue)) + ' commands to go'
				
				else:
					if debugMode:
						print '[INFO] No movements to execute'
					self.legMovementDicts = self.commandLegMovementDict['Nothing']
				return self.GetNextCommandSet()
			
			else:
				print '[ERROR] Invalid movement mode of: ' + self.moveMode + '. Returning to standing coordinates'
				return self.GetStandCommand()
		
		else:		#Execute the movement commands mapped in legMovementDicts
			legMovementDict = self.legMovementDicts[self.legMovementDictToProcess]		#Grab the dictionary
			moveVector = Vector3(0,0,0)										#Initialize default move vector
			allMovementsComplete = True										#Flag for if all movements complete
			for legSet in legMovementDict:									#Process each set of legs in the dictionary
				legs = LegMover.moveSets[legSet]							#Get legs to operate on
				movement = legMovementDict[legSet]							#Grab Movement instance for this leg set out of the dictionary
				if self.nextMotionIndex < len(movement.motion):		#If there are differential movements left	
					allMovementsComplete = False							#There was a movement this loop
# 					movement =  legMovementDict[legSet]						#Grab Movement instance for this leg set out of the dictionary
# 					moveVector = legMovementDict[legSet].motion[self.nextMotionIndex]	#Get the move vector for all legs in this set
					moveVector = movement.motion[self.nextMotionIndex]
					for leg in legs:											#Generate a command for each legt	
						if movement.coordinatePlane is 'Robot':		#Build coordinate on robot plane (+x, +y absolute)
							adjustedMoveVector = self.TransformLegDirections(leg, moveVector)		#Adjust Y coordinate appropriately
							
							#Add this new movement to the current position
							self.currentLegCoordinates[leg].AddToSelf(adjustedMoveVector)
						
						elif movement.coordinatePlane is 'Leg':			#Build coordinate on leg plane (+x is out from leg, +y is CW)
							self.currentLegCoordinates[leg].AddToSelf(moveVector)
						
						#Create a command to move to this point
						command = MoveCommand("LegMovement", {leg : self.currentLegCoordinates[leg]}, 1000)
						commandSet.append(command)		#Append to the command set to return
						
						if debugMode:
							cmdPrint = adjustedMoveVector.GetPrintString()
							print 'Leg ' + str(leg) + ' differential command: ' + cmdPrint
				
				if debugMode:		
					print 'All commands created for leg set : ' + str(legs)
					print 
			self.nextMotionIndex += 1			#Increment which motion we are doing in the differential array
			if allMovementsComplete:			#Complete whole for loop with no new movements -- go to next leg set
				if debugMode:
					print 'All movements complete'
				self.nextMotionIndex = 0
				self.legMovementDictToProcess += 1
			return commandSet			

	'''
	Return a standing command for processing by the ServoController
	'''
	def GetStandCommand(self):
		standCommand = []		#Convert to array for handling like the GetNextCommandSet function
		standCommand.append(MoveCommand("Stand", standingCoordinates, 1000))
		return standCommand
	
	'''
	Return a 90/90/90 command for processing by the ServoController
	'''
	def Get90DegreeCommand(self):
		command90Deg = []
		command90Deg.append(MoveCommand("90", ninetyCoordinates, 1000))
		return command90Deg
		
	'''
	Return a command containing only the current leg coordinates (do nothing)
	'''
	def GetEmptyCommand(self):
		print 'Info: Executing GetEmptyCommand() - please verify working correctly'
		nothingCommand = []
		nothingCommand.append(MoveCommand('Nothing', self.currentLegCoordinates, 1000))
		return nothingCommand
		
	'''
	Fix Y coordinate direction for legs on the LEFT side of the robot,
	and flip X/Y coordinates for front/back legs go match the coordinate plane of the robot.
	Pass in actual leg number (1-8), not index (0-7)
	'''
	def TransformLegDirections(self, legID, vector):
		localTransformVector = copy.copy(vector)		#Get local copy to transform
		
		#Invert Y if leg is on left side OR front/back
 		legYmultiplier = 1
 		legXmultiplier = 1
 		if legID is 5 or legID is 6 or legID is 7 or legID is 8 or legID is 1:
 			legYmultiplier = -1
 			legXmultiplier = -1
 		
 		localTransformVector.y *= legYmultiplier
 		localTransformVector.x *= legXmultiplier
 		
 		if legID is 1 or legID is 4 or legID is 8:
 			localTransformVector.x *= -1
		
		#Handle flipping x/y coordinate for front/back legs
		if legID is 1 or legID is 4 or legID is 8 or legID is 5:
			xVector = localTransformVector.x
			localTransformVector.x = localTransformVector.y
			localTransformVector.y = xVector
		
		#Note: Leg 5 is the only leg on both the back AND with an inverted Y direction.
		#Need to flip once more after transformation complete
		if legID is 5:
			localTransformVector.x *= -1
				
		return localTransformVector
	