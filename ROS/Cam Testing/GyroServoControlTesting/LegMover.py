from SpiderLeg import MoveCommand
from SpiderLeg import Vector3
import copy

'''
Implements basic forward walking motion on a flat surface
'''

#Starting point from which to base the standing coordinate
baseCoor = Vector3(20, 0, 10)	#90, 90, 90 at (13.4, 0, 21.2)

#Alternating positive/negative standing offset
splayFactor = 5		#How far legs splay apart in standing position

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
calibratedOffsets = {1 : Vector3(2,0,-18), 
			2 : Vector3(10,-12,-11),  
			3 : Vector3(-5,-5,-5),
			4 : Vector3(-4,0,-7),  
			5 : Vector3(-9,-11,8),
			6 : Vector3(-2,5,-12),
			7 : Vector3(-1,-15,-25),
			8 : Vector3(10,-9,-10)
}


#Debug & safety modes
debugMode = False					#Explicit printouts of leg positions for debugging

	#ENABLE THIS WHEN TESTING A NEW WALKING SET
safeAlgorithmLoopMode = False		#Always return to standard splay position at end of each movement loop

class LegMover:
	'''
	Contains the algorithms for moving spider legs. 
	
	Supports differential movement arrays, pre-defined or function-generated arrays
	of differential points from the current position to move the legs. On initialization,
	this object maps predefined sets of legs (moveSets) to these differential movement arrays.
	
	Each command is executed in parallel for each leg in moveSets mapped against its differential
	move array. Once completed, the next leg set : movement array is executed in parallel, etc.
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
	
	#Do nothing motion
	nothingMotion = [Vector3(0,0,0)]
	
	#Foot stomping test
	stompUpMotion = [Vector3(0,0,-5)]
	stompDownMotion = [Vector3(0,0,5)]
	
	#Above Z test
	holdUpMotion = [Vector3(0,0,-10)]
	holdDownMotion = [Vector3(0,0,10)]
	
	swingMotion = [Vector3(10,0,0), Vector3(-10,0,0)]

#Wave Gait
	#FWD
	standardFwdStepMotion = [Vector3(0,-2.5,-5), Vector3(0,-2.5,5)]
	halfFwdHump = [Vector3(0,5,0)]
	
	#REV
	standardRevStepMotion = [Vector3(0,2.5,-5), Vector3(0,2.5,5)]
	halfRevHump = [Vector3(0,-5,0)]
	
#Set down
	sitDownMotion = [Vector3(0,0,-10)]

	#For stretch test
	stretchMotion = [Vector3(0,-7.5,0)]
	
	def __init__(self, legArray):
		'''
		Args:
			legArray ([SpiderLeg]): array of SpiderLeg objects to act upon
		'''
		self.spiderLegs = legArray
		self.currentLegCoordinates = copy.deepcopy(standingCoordinates)  #Default coordinates to standing position
		self.legMovementDictToProcess = 0				#Which legMovementDict we are accessing
		self.nextMotionIndex = 0						#Which motion within each differential movement array we are at
		
		resetOnceFlag = False
		
		'''
		Setup mapping of keyboard commands to dictionary arrays for movement
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
			{6 : LegMover.standardFwdStepMotion},
			{7 : LegMover.standardFwdStepMotion},
			{8 : LegMover.standardFwdStepMotion},
			{9 : LegMover.standardFwdStepMotion},
			{10 : LegMover.standardFwdStepMotion},
			{11 : LegMover.standardFwdStepMotion},
			{12 : LegMover.standardFwdStepMotion},
			{13 : LegMover.standardFwdStepMotion},
			{14 : LegMover.halfFwdHump}
		]
		
		waveBackwardMovements = [
			{6 : LegMover.standardRevStepMotion},
			{7 : LegMover.standardRevStepMotion},
			{8 : LegMover.standardRevStepMotion},
			{9 : LegMover.standardRevStepMotion},
			{10 : LegMover.standardRevStepMotion},
			{11 : LegMover.standardRevStepMotion},
			{12 : LegMover.standardRevStepMotion},
			{13 : LegMover.standardRevStepMotion},
			{14 : LegMover.halfRevHump}
		]
		
		rippleForwardMovements = [ 
			#First half
	 		{0 : LegMover.liftUpFwdMotion},
	 		{1 : LegMover.halfFwdHump},
	 		{0 : LegMover.putDownFwdMotion},		
	
	 		#Second half
	 		{1 : LegMover.liftUpFwdMotion},
	 		{0 : LegMover.halfFwdHump},
	 		{1 : LegMover.putDownFwdMotion}	
		]
		
		rippleBackwardsMovements = [
	 		{1 : LegMover.liftUpReverseMotion},
	 		{0 : LegMover.halfRevHump},
	 		{1 : LegMover.putDownReverseMotion},
	 		
	 		{0 : LegMover.liftUpReverseMotion},
	 		{1 : LegMover.halfRevHump},
	 		{0 : LegMover.putDownReverseMotion}
		]
		
		doNothingMovements = [
			{14 : LegMover.nothingMotion}		
		]
		
		sitDownMovements = [
			{14 : LegMover.sitDownMotion},
			{14 : LegMover.nothingMotion},
			{14 : LegMover.nothingMotion},	
			{14 : LegMover.nothingMotion},	
			{14 : LegMover.nothingMotion},	
			{14 : LegMover.nothingMotion},	
			{14 : LegMover.nothingMotion},	
			{14 : LegMover.nothingMotion},	
			{14 : LegMover.nothingMotion},	
			{14 : LegMover.nothingMotion},	
			{14 : LegMover.nothingMotion},	
			{14 : LegMover.nothingMotion},	
			{14 : LegMover.nothingMotion},	
			{14 : LegMover.nothingMotion},	
			{14 : LegMover.nothingMotion},	
			{14 : LegMover.nothingMotion},	
			{14 : LegMover.nothingMotion},	
			{14 : LegMover.nothingMotion},	
			{14 : LegMover.nothingMotion},	
			{14 : LegMover.nothingMotion},	
			{14 : LegMover.nothingMotion},	
			{14 : LegMover.nothingMotion}	
		]
		
		self.commandLegMovementDict = {
			'Nothing' : doNothingMovements,
			'Forward' : rippleForwardMovements,
			'Back' : rippleBackwardsMovements,
			'Down' : sitDownMovements
		}
		
		self.legMovementDicts = doNothingMovements
		
		#Reset to standing once after new command flag
		self.resetOnceFlag = False
		self.standingMode = True
	
	'''
	Set the current movement command out of the pre-defined movements (see init)
	Command: String of form
		Forward, Back, Left, Right, CW, CCW, Up, Down, Stand
	'''
	def SetMovementCommand(self, command):
		print command
		self.resetOnceFlag = True	
		if command == 'Stand':
			print 'Robot returning to standing motion'
			self.standingMode = True
		else:
			self.standingMode = False
		
			#Try finding a match between this command and the dictionary of available commands
			try:
				self.legMovementDicts = self.commandLegMovementDict[command]
				print 'Received move command'
			except:
				print 'WARNING: Invalid movement command sent to robot. Freezing'
				self.legMovementDicts = self.commandLegMovementDict['Nothing']
	
	'''
	Returns a series of commands for the next leg movement
	Use nextMotionIndex value to determine which legs to move,
	indexing into movement array using nextMovementVectorIndex
	'''
	def GetNextCommandSet(self):
		commandSet = []		 #Array of MoveCommand objects to return
		
		if debugMode:
			print 'Current leg coordinates:'
			print self.currentLegCoordinates
			for leg in self.currentLegCoordinates:
				self.currentLegCoordinates[leg].Print()
			print
		
		#Stand up mode
		if self.standingMode:
			print 'I am standing'
			self.currentLegCoordinates = copy.deepcopy(standingCoordinates)
			return self.GetStandCommand()
		
		#Stand coordinates for transitioning between commands
		elif self.resetOnceFlag:
			print 'Resetting to standing for new command'
			self.currentLegCoordinates = copy.deepcopy(standingCoordinates)
			self.resetOnceFlag = False
			return self.GetStandCommand()
		
		#Is the movement dictionary fully processed?
		if self.legMovementDictToProcess >= len(self.legMovementDicts):
			self.nextMotionIndex = 0				#Set flag to begin algorithm over again
			self.legMovementDictToProcess = 0		#Set flag to begin at top of the first differential movement array
			
			if safeAlgorithmLoopMode:	#Reset to standing position
				print 'Reset to standing position'
				self.currentLegCoordinates.clear()
				self.currentLegCoordinates = copy.deepcopy(standingCoordinates)		#Create deep (by-value) copy and replace current dictionary
				return self.GetStandCommand()	#Reset to normal standing position
			else:
				return self.GetNextCommandSet()
		
		else:		#Execute the movement commands mapped in legMovementDicts
			legMovementDict = self.legMovementDicts[self.legMovementDictToProcess]		#Grab the dictionary
			moveVector = Vector3(0,0,0)										#Initialize default move vector
			allMovementsComplete = True										#Flag for if all movements complete
			for legSet in legMovementDict:									#Process each set of legs in the dictionary
				legs = LegMover.moveSets[legSet]							#Get legs to operate on
				if self.nextMotionIndex < len(legMovementDict[legSet]):		#If there are differential movements left	
					allMovementsComplete = False							#There was a movement this loop
					moveVector = legMovementDict[legSet][self.nextMotionIndex]	#Get the move vector for all legs in this set
					for leg in legs:											#Generate a command for each legt	
						adjustedMoveVector = self.TransformLegDirections(leg, moveVector)		#Adjust Y coordinate appropriately
						
						#Add this new movement to the current position
						self.currentLegCoordinates[leg].AddToSelf(adjustedMoveVector)
						
						#Create a command to move to this point
						command = MoveCommand("LegMovement", {leg : self.currentLegCoordinates[leg]}, 1000)
						commandSet.append(command)		#Append to the command set to return
						
						if debugMode:
							cmdPrint = adjustedMoveVector.GetPrintString()
							print 'Leg ' + str(leg) + ' differential command: ' + cmdPrint
				
				if debugMode:		
					print 'All commands created for leg set : ' + str(legs)
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
	Fix Y coordinate direction for legs on the LEFT side of the robot,
	and flip X/Y coordinates for front/back legs go match the coordinate plane of the robot
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
	