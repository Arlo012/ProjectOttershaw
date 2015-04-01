from SpiderLeg import MoveCommand
from SpiderLeg import Vector3
import copy

'''
Implements basic forward walking motion on a flat surface
'''

#90, 90, 90 at (13.4, 0, 21.2)
baseCoor = Vector3(13, 0, 13)

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
			5 : Vector3(-9,-11,10),
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
	moveSets = [ [2,6], [3,7],		  # Sides by leg ID
				 [1,5], [4,8] ]		# Front/back by leg ID	 
  
#Differential movement arrays
	#Order of movement, by right leg (must reverse Y for left side)
	liftForwardMotion = [Vector3(0,0,-5), Vector3(0,-5,0), Vector3(0,0,5)]	#First up and over, then down and over more
	rotateMotion = [Vector3(0,5,0), Vector3(0,0,0)]

	def __init__(self, legArray):
		'''
		Args:
			legArray ([SpiderLeg]): array of SpiderLeg objects to act upon
		'''
		self.spiderLegs = legArray
		self.currentLegCoordinates = copy.deepcopy(standingCoordinates)  #Default coordinates to standing position
		self.nextPairToMove = 0									#Index 0-3 for leg pairs
		self.nextMovementVectorIndex = 0					   #Index into leg movement array, e.g. sideLegMovement
		
		#Mapping of leg set index to differential movement arrays. Execute these serially
		self.legMovementDicts = [ 
			{0 : LegMover.liftForwardMotion, 3 : LegMover.liftForwardMotion},		#Lift leg 2/6 and move forward
			{1 : LegMover.rotateMotion, 2 : LegMover.rotateMotion},					#Rotate
		]
		
	'''
	Returns a series of commands for the next leg movement
	Use nextPairToMove value to determine which legs to move,
	indexing into movement array using nextMovementVectorIndex
	'''
	def GetNextCommandSet(self):
		commandSet = []		 #Array of MoveCommand objects to return
		#print 'Current leg coordinates:'
		#print self.currentLegCoordinates
		#for leg in self.currentLegCoordinates:
		#	self.currentLegCoordinates[leg].Print()
		#print
		#return self.GetStandCommand()


		#Check if each leg set has completed all of its movements
		if self.nextPairToMove >= len(self.legMovementDicts):
			self.nextPairToMove = 0				#Set flag to begin algorithm over again
			self.nextMovementVectorIndex = 0	#Set flag to begin at top of the next differential movement array
			
			#Reset to standing position
			self.currentLegCoordinates.clear()
			self.currentLegCoordinates = copy.deepcopy(standingCoordinates)		#Create deep (by-value) copy and replace current dictionary
			
			return self.GetStandCommand()	#Reset to normal standing position
		
		else:		#Execute the movement commands mapped in legMovementDicts
			
			for legSetIndex in self.legMovementDicts[self.nextPairToMove]:	#Access the keys (mapping of leg index) against their movement vector
				moveVector = Vector3(0,0,0)
				
				#Grab the dictionary mapping legSetIndex : legMovement, populate moveVector
				moveDictionary =  self.legMovementDicts[self.nextPairToMove]
				for legSet in moveDictionary:		#For every mapping between a leg set index and a differential movement list
					if self.nextMovementVectorIndex >= len(moveDictionary[legSet]):		#Completed all differential movements -- go to next leg set
						self.nextPairToMove += 1
						self.nextMovementVectorIndex = 0
						
						#for leg in standingCoordinates:
						#	self.currentLegCoordinates[leg] = standingCoordinates[leg]		#Return all legs to standing position
						return self.GetNextCommandSet()
					else:
						moveVector = moveDictionary[legSet][self.nextMovementVectorIndex]
				
				#Associate legSetIndex with an actual leg
				legs = LegMover.moveSets[legSetIndex]		#Get leg set to operate on
				for leg in legs:
					adjustedMoveVector = self.TransformLegDirections(leg, moveVector)		#Mirror Y, flip X/Y for front/back

					#Add this new movement to the current position
					self.currentLegCoordinates[leg].AddToSelf(adjustedMoveVector)
					
					#Create a command to move to this point
					command = MoveCommand("LegMovement", {leg : self.currentLegCoordinates[leg]}, 1000)
					commandSet.append(command)		#Append to the command set to return
				
				self.nextMovementVectorIndex += 1
				return commandSet

	
	def GetStandCommand(self):
		standCommand = []		#Convert to array for handling like the GetNextCommandSet function
		standCommand.append(MoveCommand("Stand", standingCoordinates, 1000))
		return standCommand
	
	def Get90DegreeCommand(self):
		command90Deg = []
		command90Deg.append(MoveCommand("90", ninetyCoordinates, 1000))
		return command90Deg
		
	def TransformLegDirections(self, legID, vector):
		localTransformVector = copy.copy(vector)		#Get local copy to transform
		
		#Invert Y if leg is on left side
		legYmultiplier = 1
		if legID <= 4:
			legYmultiplier = 1
		else:
			legYmultiplier = -1
		
		localTransformVector.y *= legYmultiplier
		
		#Handle flipping x/y coordinate for front/back legs
		if legID is 1 or legID is 4 or legID is 5 or legID is 8:
			xVector = localTransformVector.x
			localTransformVector.x = localTransformVector.y
			localTransformVector.y = xVector
		
		return localTransformVector
	