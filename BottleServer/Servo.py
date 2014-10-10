import RPIO.PWM as PWM
import time
import math

class servo:
    '''A servo, and associated methods of controlling it'''
    
    def __init__(self, myLocation, myPin, uniqueID):
        self.location = str(myLocation)
        self.rotation = 0
        self.pin = int(myPin)
        self.ID = int(uniqueID)
        if not isSetup:
            PWMsetup()
        
    def rotate(self, _angle):
        '''Rotate this servo by a given angle'''
        angle = int(_angle)
        pwmAngle = 0
        try:
            if abs(angle) > 180:
                raise Exception('Out of bounds servo rotation')
            else: 
                pwmAngle = int(1000 + (angle/180) * 1000)
                print("PWM timer of " + str(pwmAngle))
                PWM.add_channel_pulse(1, self.pin, 0, pwmAngle)
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
    
    
# Static stuff below here, outside the class

isSetup = False

def PWMsetup():
    '''Call me to setup servo channel at start of program'''
    
    #--------------------------------------------------
    # set up PWM pulse increment to 1us
    #--------------------------------------------------
    PWM.setup(1)
    PWM.init_channel(1, 20000)
    isSetup = True
    
def PWMcleanup():
    '''Call me at servo shutdown'''
    PWM.clear_channel(1)
    PWM.cleanup()
    isSetup = False
