import math

class SpiderLeg:
    '''
    Contains:
        servo objects (x3)
        int location (leg count), counted clockwise from top
    '''
    
    L_Thigh = 13 #length of thigh #dummy variable
    L_Calf = 21 
    
    def __init__(self, ID, servo1, servo2, servo3):
        self.legID = ID        #0 - 7 clockwise for leg ID
        self.servo1 = servo1
        self.servo2 = servo2
        self.servo3 = servo3
        
    '''
    Pass in Vector 3 of desired leg location
    '''
    def GetAngles(self, desiredLocation):
        #Consider the origin to be the shoulder joint
        #Desired location can be a composite of int x and int y.
        #We can find the distance to that point as follows
        
        x = desiredLocation.x
        y = desiredLocation.y
        z = desiredLocation.z
        
        #Calculations for theta3, knee 
        H =  math.sqrt(z**2 + x**2) #H = distanceFrom(0,0 to destination)
        numerator1 = SpiderLeg.L_Thigh**2 + SpiderLeg.L_Calf**2 - H**2
        denominator1 = 2 * SpiderLeg.L_Thigh * SpiderLeg.L_Calf
        theta3 = 180 - 180/math.pi * math.acos(numerator1/denominator1)
        
        #Calculations for theta2, shoulder
        numerator2 = SpiderLeg.L_Thigh**2 + H**2 - SpiderLeg.L_Calf**2
        denominator2 = 2 * SpiderLeg.L_Thigh * H
        theta2 = 180 - 180/math.pi * math.acos(numerator2/denominator2) - (180/math.pi * math.atan2(x,z))
        
        
        #Calculations for theta1, shoulder y direction
        theta1 = 180/math.pi * math.asin((y/math.sqrt(x**2 + y**2))) + 90
        
        #Return the three angles that this leg's servos need to move to
        return Vector3(theta1, theta2, theta3)
        
        
        
class Servo:
    '''
    Contains:
        string location
        int ID, corresponds loosely to arduino analog pin
        float currentAngle
        float desiredAngle
    '''
    servoID = 0     #Class counter for each new servo
    
    def __init__(self, location, currentAngle, desiredAngle): 
        self.location = location
        self.currentAngle = currentAngle
        self.desiredAngle = desiredAngle
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
