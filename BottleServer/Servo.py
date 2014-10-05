import RPIO.PWM as PWM
import time
import math

class servo:
    '''A servo, and associated methods of controlling it'''
    
    def __init__(self, myLocation, myPin):
        self.location = myLocation
        self.rotation = 0
        self.pin = myPin
        
    def rotate(self, angle):
        '''Rotate this servo by a given angle'''
        pwmAngle = 0
        try:
            if abs(angle) > 180:
                raise Exception('Out of bounds servo rotation')
            else: 
                pwmAngle = 1000 + (angle/180) * 1000
                PWM.add_channel_pulse(1, pin, 0, pwmAngle)
        except Exception as detail:
            print(detail)
            
    def setPin(self, newPin):
        '''
        Select pin for servo PWM output (BCM numbering)
        See http://raspberrypi.stackexchange.com/questions/12966/what-is-the-difference-between-board-and-bcm-for-gpio-pin-numbering
        '''
        self.pin = newPin
    
    
# Static stuff below here, outside the class

def PWMsetup():
    '''Call me to setup servo channel at start of program'''
    
    #--------------------------------------------------
    # set up PWM pulse increment to 1us
    #--------------------------------------------------
    PWM.setup(1)
    
def PWMcleanup():
    '''Call me at servo shutdown'''
    PWM.clear_channel(1)
    PWM.cleanup()