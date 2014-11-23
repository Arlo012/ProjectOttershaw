import time
import math
import SerialCommunicator

class servo:
    '''A servo, and associated methods of controlling it'''
    
    uniqueID = 0
    
    def __init__(self, myLocation):
        self.location = str(myLocation)
        self.rotation = 0
        self.ID = uniqueID
        uniqueID += 1
        if not isSetup:
            PWMsetup()
            
        self.commInstance = SerialComm.Instance()
        
    def rotate(self, _angle):
        '''Rotate this servo by a given angle'''
        angle = int(_angle)
        pwmAngle = 0
        try:
            if abs(angle) > 180:
                raise Exception('Out of bounds servo rotation')
            else:
                self.commInstance

        except Exception as detail:
            print(detail)
            
    def setPin(self, newPin):
        '''
        Select pin for servo PWM output (BCM numbering)
        See http://raspberrypi.stackexchange.com/questions/12966/what-is-the-difference-between-board-and-bcm-for-gpio-pin-numbering
        '''
        self.pin = newPin
        
    def getPin(self):
        return self.pin
    
    def getLocation(self):
        return self.location
    
    def getID(self):
        return self.ID
    