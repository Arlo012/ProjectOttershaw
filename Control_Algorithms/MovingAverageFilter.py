import time

class AnalogTimeReadings:
	'''
	New data type with time-based readings. Allows implementation of PID
	algorithms for sensor dataReadings
	'''

	def __init__(self, value):
		self.readTime = time.time()
		self.readValue = value

class MovingAverageFilter:
	'''
	Implementation of moving average filter of N data points
	Assume that when robot is initialized there is nothing in front of it
	Distance = sonic reading max value = 250 cm 
	'''

	def __init__(self, dataPoints, derivativePoints, dummyVal):
		'''
		Initiate filter with N sample points and fill in 
		the list with dummy readings
		'''
		self.dataReadings = []
		self.derivativeValues = []
		self.averageDataValue = dummyVal
		self.averageDerivativeValue = 0
		self.sampleSize = dataPoints
		self.derivativeSampleSize = derivativePoints
		dummyAnalogVal = AnalogTimeReadings(dummyVal)

		# Test if works
		fillArray = lambda size, val, array: [self.array.append(val) for i in range(0,size)]
		fillArray(self.sampleSize,dummyAnalogVal,dataReadings)
		fillArray(self.derivativeSampleSize,0,derivativeValues)

		# for i in range(0, self.sampleSize):
		# 	self.dataReadings.append(dummyAnalogVal)

		# for i in range(0, self.derivativeSampleSize):
		# 	self.derivativeValues.append(0)

	def updateDataFilter(self, data):
		'''
		Remove the first reading, add the new data point,
		and return the new average
		'''
		self.dataReadings.pop(0)
		self.dataReadings.append(data)

		#Computation of new Average Value
		self.averageDataValue = sum([AnalogTimeReadings.readValue for AnalogTimeReadings in dataReadings])/self.sampleSize
		# for i in range(0,self.sampleSize):
		# 	self.averageDataValue = self.averageDataValue + self.dataReadings[i].readValue
		# self.averageDataValue /= self.sampleSize

	def updateDerivativeFilter(self, data):
		'''
		Remove the first reading, add the new data point,
		and return the new average
		'''
		self.derivativeValues.pop(0)
		self.derivativeValues.append(data)

		#Computation of new Average Value
		self.averageDerivativeValue = sum(derivativeValues)/self.derivativeSampleSize
		# for i in range(0,self.derivativeSampleSize):
		# 	self.averageDerivativeValue = self.averageDerivativeValue + self.derivativeValues[i]
		# self.averageDerivativeValue /= self.sampleSize

	'''
	Used for debugging plots -- might be useful
	'''
	# def getDataTimes(self):
	# 	tempArray = []
	# 	for i in range(0,self.sampleSize):
	# 		tempArray.append(self.dataReadings[i].readTime)
	# 	return tempArray

	# def getDataValues(self):
	# 	tempArray = []
	# 	for i in range(0,self.sampleSize):
	# 		tempArray.append(self.dataReadings[i].readValue)
	# 	return tempArray