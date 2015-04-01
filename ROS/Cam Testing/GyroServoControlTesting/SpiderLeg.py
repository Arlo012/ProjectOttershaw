import math

class SpiderLeg:
    '''
    Contains:
        servo objects (x3)
        int location (leg count), counted clockwise from top
    '''
    
    L_Thigh = 13.4     #length of thigh 
    L_Calf = 21.2
    
    def __init__(self, ID, servo1, servo2, servo3):
        self.legID = ID        #0 - 7 clockwise for leg ID
        self.servo1 = servo1
        self.servo2 = servo2
        self.servo3 = servo3
        
    '''
    Use the desired point parameter set in constructor
    '''
    def GetAngles(self):
        #Consider the origin to be the shoulder joint
        #Desired location can be a composite of int x and int y.
        #We can find the distance to that point as follows
        
        x = self.desiredPoint.x
        y = self.desiredPoint.y
        z = self.desiredPoint.z
        
        #self.desiredPoint.Print()
        
        G = math.sqrt(x**2 + y**2)
        H = math.sqrt(z**2 + G**2)
        #theta1 = 180/math.pi * math.acos(y/math.sqrt(x**2 + y**2))
        theta1 = 180/math.pi*math.acos(y/G)
        #H =  math.sqrt(z**2 + x**2) / math.cos(math.pi/180 * (90-theta1)) #H = distanceFrom(0,0 to destination)
        
        #Calculations for theta2, shoulder
        numerator2 = SpiderLeg.L_Thigh**2 + H**2 - SpiderLeg.L_Calf**2
        denominator2 = 2 * SpiderLeg.L_Thigh * H
        theta2 = 180 - 180/math.pi * math.acos(numerator2/denominator2) - (180/math.pi * math.atan2(G,z))
        
         #Calculations for theta3, knee 
        numerator1 = SpiderLeg.L_Thigh**2 + SpiderLeg.L_Calf**2 - H**2
        denominator1 = 2 * SpiderLeg.L_Thigh * SpiderLeg.L_Calf
        theta3 = 180 - 180/math.pi * math.acos(numerator1/denominator1)
        
        #Return the three angles that this leg's servos need to move to
        
        return Vector3(theta1, theta2, theta3)
    
    '''
    Update new desired position (vector3) for the legs
    '''
    def UpdateDesiredPosition(self, position):
        self.desiredPoint = position
        
    '''
    Return the point in 3D space this leg wants to move to
    '''
    def GetDesiredPosition(self):
        return self.desiredPoint
        
        
class Servo:
    '''
    Contains:
        string location
        int ID, corresponds loosely to arduino analog pin
        float currentAngle
        float desiredAngle
    '''
    servoID = 0     #Class counter for each new servo
    
    def __init__(self, location, desiredAngle): 
        self.location = location
        self.angle = desiredAngle
        self.SetID()
        
    def SetID(self):
        self.ID = Servo.servoID
        Servo.servoID = Servo.servoID + 1
        
class Vector3:
    '''
    A 3D vector (x,y,z)
    '''
    
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    
    '''
    Add two arbitrary vectors, return result
    '''
    def Add(vector1, vector2):
        returnVector = Vector3(vector1.x + vector2.x, vector1.y + vector2.y, vector1.z + vector2.z)
        return returnVector
    
    '''
    Add another vector to this one
    '''
    def AddToSelf(self, vector):
        self.x += vector.x
        self.y += vector.y
        self.z += vector.z
    
    def Print(self):
        print str(self.x) + ", " + str(self.y) + ", " + str(self.z)
        
class MoveCommand:
    '''
    A series of coordinates for each leg to move.
    Execute these serially with delay to allow their execution
    
    Contains:
        string name
        coordinatesToMove: dictionary of leg ID : coordinates to move (Vector3)
        int executionTime: milliseconds to move to this position
    '''
    
    def __init__(self, name, coordinateArray, executionTime):
        self.commandName = name
        self.coordinatesToMove = coordinateArray      #Dictionary leg # : Vector3 movement
        self.timeToExecute = executionTime
        
    '''
    Returns array of 8 leg coordinates for this position
    '''
    def GetCoordinates(self):
        return self.coordinateArray
    
    def GetCmdName(self):
        return self.commandName
    
