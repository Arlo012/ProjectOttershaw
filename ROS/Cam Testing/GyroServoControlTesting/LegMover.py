from SpiderLeg import MoveCommand
from SpiderLeg import Vector3
import copy

'''
Implements basic forward walking motion on a flat surface
'''

#90, 90, 90 at (13.4, 0, 21.2)
baseCoor = Vector3(13, 0, 15)

#Alternating positive/negative standing offset
splayFactor = 5		#How far legs splay apart in standing position

standingCoordinates = {1: Vector3.Add(baseCoor, Vector3(0,splayFactor,0)),#Leg1
			2 : Vector3.Add(baseCoor, Vector3(0,-splayFactor,0)),   #Leg2
			3 : Vector3.Add(baseCoor, Vector3(0,splayFactor,0)),	#Leg3
			4 : Vector3.Add(baseCoor, Vector3(0,-splayFactor,0)),   #Leg4
			5 : Vector3.Add(baseCoor, Vector3(0,splayFactor,0)),	#Leg5
			6 : Vector3.Add(baseCoor, Vector3(0,-splayFactor,0)),   #Leg6
			7 : Vector3.Add(baseCoor, Vector3(0,splayFactor,0)),	#Leg7
			8 : Vector3.Add(baseCoor, Vector3(0,-splayFactor,0))	#Leg8		
}

ninetyCoordinates = {1: Vector3(13,0,21),#Leg1
			2: Vector3(13,0,21),   #Leg2
			3: Vector3(13,0,21),	#Leg3
			4: Vector3(13,0,21),   #Leg4
			5: Vector3(13,0,21),	#Leg5
			6: Vector3(13,0,21),   #Leg6
			7: Vector3(13,0,21),	#Leg7
			8: Vector3(13,0,21)	#Leg8		
}

#These are the calibrated offsets for 90 90 90 (with subtractions for offset to spread)
calibratedOffsets = {1 : Vector3(2,0,-18), 
			2 : Vector3(10,-12,-11),  
			3 : Vector3(-5,-5,-5),
			4 : Vector3(-4,0,-7),  
			5 : Vector3(-9,-11,8),
			6 : Vector3(-2,5,-12),
			7 : Vector3(-1,-15,-25),
			8 : Vector3(10,-9,-10)
}

class LegMover:
	'''
	Contains the algorithms for moving spider legs. 
	
	Supports differential movement arrays, pre-defined or function-generated arrays
	of differential points from the current position to move the legs. On initialization,
	this object maps predefined sets of legs (moveSets) to these differential movement arrays.
	
	Each command is executed in parallel for each leg in moveSets mapped against its differential
	move array. Once completed, the next leg set : movement array is executed in parallel, etc.
	'''
	
#Sets for moving together in the walking algorithm
	#moveSets = [ [2,6], [3,7],		  # Sides by leg ID
	#			 [1,5], [4,8] ]		# Front/back by leg ID	 
	moveSets = [ 
			[1,3,5,7], [2,4,6,8],	#(0,1)Standard walking sets
			[2,6], [3,7],		  	#(2,3)Sides by leg ID
			[1,5], [4,8], 			#(4,5) Front/back by leg ID	
			[1], [2], [3], [4],		#(6-9)
			[5], [6], [7], [8],		#(10-13)
			[1,2,3,4,5,6,7,8]		#(14)
			 ]
  
#Differential movement arrays
	#Order of movement, by right leg (must reverse Y for left side)
	#liftForwardMotion = [Vector3(0,0,-5), Vector3(0,-5,0), Vector3(0,0,5), Vector3(0,0,0)]	#First up and over, then down and over more
	liftUpMotion = [Vector3(0,-2.5,-5)]
	putDownMotion= [Vector3(0,-2.5,5)]
	rotateMotion = [Vector3(0,5,0)]
	holdUpMotion = [Vector3(0,0,-5)]
	holdDownMotion = [Vector3(0,0,5)]
	
	swingMotion = [Vector3(10,0,0), Vector3(-10,0,0)]

	#For forward wave gait
	standardFwdStepMotion = [Vector3(0,-2.5,-5), Vector3(0,-2.5,5)]
	halfFwdHump = [Vector3(0,5,0)]
	
	#For reverse wave gait
	standardRevStepMotion = [Vector3(0,2.5,-5), Vector3(0,2.5,5)]
	halfRevHump = [Vector3(0,-5,0)]
	
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
		
		
		#Mapping of leg set index to differential movement arrays. Execute these serially
		#Everything in one mapping will be executed in parallel and bunched into one commandSet[] for execution
		self.legMovementDicts = [ 
			#SWING TEST
# 			{6 : LegMover.holdUpMotion},
# 			{6 : LegMover.swingMotion},
# 			{6 : LegMover.swingMotion},
# 			{6 : LegMover.swingMotion},
# 			{6 : LegMover.swingMotion},
# 			{6 : LegMover.swingMotion},
# 			{6 : LegMover.swingMotion},
# 			{6 : LegMover.swingMotion},
# 			{6 : LegMover.swingMotion},
# 			{6 : LegMover.swingMotion},
# 			{6 : LegMover.swingMotion},
# 			{6 : LegMover.swingMotion},
# 			{6 : LegMover.swingMotion},
# 			{6 : LegMover.holdDownMotion}

			#UP/DOWN TEST
# 			{14 : LegMover.holdUpMotion},
# 			{14 : LegMover.holdDownMotion}
			
			#FORWARD WAVE GAIT
# 			{6 : LegMover.standardFwdStepMotion},
# 			{7 : LegMover.standardFwdStepMotion},
# 			{8 : LegMover.standardFwdStepMotion},
# 			{9 : LegMover.standardFwdStepMotion},
# 			{10 : LegMover.standardFwdStepMotion},
# 			{11 : LegMover.standardFwdStepMotion},
# 			{12 : LegMover.standardFwdStepMotion},
# 			{13 : LegMover.standardFwdStepMotion},
# 			{14 : LegMover.halfFwdHump}

			#REVERSE WAVE GAIT
# 			{6 : LegMover.standardRevStepMotion},
# 			{7 : LegMover.standardRevStepMotion},
# 			{8 : LegMover.standardRevStepMotion},
# 			{9 : LegMover.standardRevStepMotion},
# 			{10 : LegMover.standardRevStepMotion},
# 			{11 : LegMover.standardRevStepMotion},
# 			{12 : LegMover.standardRevStepMotion},
# 			{13 : LegMover.standardRevStepMotion},
# 			{14 : LegMover.halfRevHump}
			
			#4/5 DEBUG
# 			{6 : LegMover.standardStepMotion},
# 			{13 : LegMover.standardStepMotion},
# 			{9 : LegMover.standardStepMotion},
# 			{10 : LegMover.standardStepMotion},
# 			{9 : LegMover.halfHump, 10 : LegMover.halfHump, 6 : LegMover.halfHump, 13 : LegMover.halfHump}

			#STRETCH TEST
# 			{14 : LegMover.stretchMotion}
			
			#QUADROPOD WALK ALGORITHM
			{0 : LegMover.liftUpMotion},
			{1 : LegMover.rotateMotion},
			{0 : LegMover.putDownMotion},		
			
			#Second half
			{1 : LegMover.liftUpMotion},
			{0 : LegMover.rotateMotion},
			{1 : LegMover.putDownMotion}	
		]
		
	'''
	Returns a series of commands for the next leg movement
	Use nextMotionIndex value to determine which legs to move,
	indexing into movement array using nextMovementVectorIndex
	'''
	def GetNextCommandSet(self):
		commandSet = []		 #Array of MoveCommand objects to return
		#print 'Current leg coordinates:'
		#print self.currentLegCoordinates
		#for leg in self.currentLegCoordinates:
		#	self.currentLegCoordinates[leg].Print()
		#print
		return self.GetStandCommand()
		
		if self.legMovementDictToProcess >= len(self.legMovementDicts):
			self.nextMotionIndex = 0				#Set flag to begin algorithm over again
			self.legMovementDictToProcess = 0	#Set flag to begin at top of the next differential movement array
			
			#Reset to standing position
			self.currentLegCoordinates.clear()
			self.currentLegCoordinates = copy.deepcopy(standingCoordinates)		#Create deep (by-value) copy and replace current dictionary
			#print 'Reset to standing position'
			
			#return self.GetStandCommand()	#Reset to normal standing position
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
						
						cmdPrint = adjustedMoveVector.GetPrintString()
						#print 'Leg ' + str(leg) + ' differential command: ' + cmdPrint
						
				#print 'All commands created for leg set : ' + str(legs)
			self.nextMotionIndex += 1			#Increment which motion we are doing in the differential array
			if allMovementsComplete:			#Complete whole for loop with no new movements -- go to next leg set
				#print 'All movements complete'
				self.nextMotionIndex = 0
				self.legMovementDictToProcess += 1
			return commandSet			

	
	def GetStandCommand(self):
		standCommand = []		#Convert to array for handling like the GetNextCommandSet function
		standCommand.append(MoveCommand("Stand", standingCoordinates, 1000))
		return standCommand
	
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
		
		if legID is 5:
			localTransformVector.x *= -1
# 		if legID is 5 or legID is 4:
# 			print legID
# 			localTransformVector.Print()
		
		return localTransformVector
	