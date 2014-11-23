import threading
import time
import collections
import Servo

class sonicSensor:
    '''A control class for a sonic sensor'''
    
    uniqueID = 0
    
    #TODO: Consider re-using this code for all arduino digital ins with serial comm protocol
    def __init__(self, myLocation, actuatorServo):
        '''
        myLocation - string identifier for where the sensor is located
        uniqueID - optional: integer identifier for this sensor
        actuator - pointer to servo class that controls the sonic sensor head
        '''
        
        self.location = str(myLocation)
        self.rotation = 0   #Reset eye level
        self.ID = uniqueID
        uniqueID += 1
        
        self.actuator = actuatorServo
        
        #How many rotation angles per sweep
        self.rotationSteps = 20
        self.currentRotation = 0
        
        #Setup array of scanned distances
        self.vision = visionArray(20)    
        
        if type(actuator) is not servo:
            print("ERROR: Invalid servo object passed into sonic sensor setup")
            print("Failed to setup sonic scanner. ID: " + uniqueID + ", Location: " + location)
            
        else:
            if not setup:
                sonicSetup()
            
    def setArduinoPin(self, newPin):
        ''' Set new arduino pin for input'''
        self.pin = newPin
        
    def getArduinoPin(self):
        '''Return arduino pin of this sensor '''
        return self.arduinoPin
    
    def getLocation(self):
        '''Return string location of this sensor '''
        return self.location
    
    def getID(self):
        '''Return unique integer ID of this sensor '''
        return self.ID
    
    def getRange(self):
        pass
        #TODO get range using serial comm
    
    def rotateToNext(self):
        ''' Rotate the sonic scanner to next designated angle '''
        self.actuator.rotate(currentRotation/rotationSteps * 90)
        self.currentRotation += 1
        if(self.currentRotation > self.rotationSteps):
            self.currentRotation = 0
    
    def sonicSetup():
        #Reset actuator
        actuator.rotate(0)
        setup = True
    
class scanThread(threading.Thread):
    '''
    This class acts as the 'head' movement for the robot
    and swivels continuously scanning using the sonic scanner
    '''
    def __init__(self, threadID, mySonicSensor):
        threading.Thread.__init__(self)
        self.sensor = mySonicSensor
        self.threadID = threadID
        
    def run(self):
        print("Starting scan protocol 1") 
        runScan()
        print("Ending scan protocol 1")

    def runScan(self):
        '''Loop scanning until scan exit flag detected'''
        while not scanExitFlag:
            self.sensor.rotateToNext()
            time.sleep(1)
            point = self.sensor.getRange()
            time.sleep(0.25)
            
            #Use visionArray class to manage data container
            self.sensor.vision.addData(point)
        thread.exit()

class visionArray:
    '''Data container of vision data points from the sonic sensor'''
    
    def __init__(self, maxPointsToStore):
        self.pointData = collections.deque()
        
        #How many data points per sweep do we want to keep?
        self.storageLength = maxPointsToStore
        
    def getData(self):
        return self.pointData
    
    def addData(self, point):
        '''Add a data point to the array while maintaining size'''
        if(len(self.pointData) > self.storageLength):
            #Remove a data point from beginning of dequeue
            self.pointData.popleft()
        self.pointData.append()
        
            

# Static stuff below here, outside the class

setup = False
scanExitFlag = False

#Begin test case
actuator = Servo("Head")
scanner = SonicScan("testLocation", actuator)
