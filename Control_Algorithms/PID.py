#The recipe gives simple implementation of a Discrete Proportional-Integral-Derivative (PID) controller. PID controller gives output value for error between desired reference input and measurement feedback to minimize error value.
#More information: http://en.wikipedia.org/wiki/PID_controller
#
#cnr437@gmail.com
#
#######    Example    #########
#
#p=PID(3.0,0.4,1.2)
#p.setPoint(5.0)
#while True:
#     pid = p.update(measurement_value)
#
#
###### Modified 2014 #######
# Jeff Eitel & The Project Ottershaw team
# Stevens Institute of Technology
# arlo012@gmail.com

import time

class PID:
    """
    Discrete PID control
    """

    def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.Derivator=Derivator
        self.Integrator=Integrator
        self.Integrator_max=Integrator_max
        self.Integrator_min=Integrator_min

        self.set_point=0.0
        self.error=0.0

    def update(self,current_value):
        """
        Calculate PID output value for given reference input and feedback
        """

        self.error = self.set_point - current_value

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.Derivator)
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        PID = self.P_value + self.I_value + self.D_value

        return PID

    def setPoint(self,set_point):
        """
        Initilize the setpoint of PID
        """
        self.set_point = set_point
        self.Integrator=0
        self.Derivator=0

    def setIntegrator(self, Integrator):
        self.Integrator = Integrator

    def setDerivator(self, Derivator):
        self.Derivator = Derivator

    def setKp(self,P):
        self.Kp=P

    def setKi(self,I):
        self.Ki=I

    def setKd(self,D):
        self.Kd=D

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.error

    def getIntegrator(self):
        return self.Integrator

    def getDerivator(self):
        return self.Derivator

#TODO get the PID control function working here
class fakeLeg:
    """Debug working of PID controller"""
    #Desire an angle of 0 degrees
    currentAngle = 0
    
    #Destination angle, time to reach it, and whether it has reached it already
    angleToReach = 0
    timeCompleted = -5
    totalTimeToMove = 0
    complete = True     #TODO necessary variable?
    
    def __init__(self, startOffBalancePoint = -45):
        self.currentAngle = startOffBalancePoint
        
    '''Move the emulated servo assuming 0.5 seconds for full 90 degree rotation'''
    def moveEmulatedServo(self, degree):
        self.moveStartTime = time.time()
        self.totalTimeToMove = abs(self.currentAngle - degree)/90 * 0.5 #in seconds
        self.timeCompleted = time.time() +  self.totalTimeToMove
        self.angleToReach = degree
        #print("Moving from " + str(self.currentAngle) + " to " + str(self.angleToReach))
        #print("Will move to destination in " + str(self.totalTimeToMove) + " seconds")
        self.complete = False
           
    def getLocation(self):
        if time.time() - self.timeCompleted > 0:
            #print("Final angle was reached")
            self.currentAngle = self.angleToReach
            self.complete = True
        else:
            #print("Final angle was not reached")
            
            # This is a confusing formula, but it works out. Moved angle as proportion of time moving over total time 
            self.angleMoved = (1 - (self.timeCompleted - time.time()) / self.totalTimeToMove) * (self.angleToReach - self.currentAngle)
            #print("Moved total of " + str(self.angleMoved) + " degrees")
            self.currentAngle += self.angleMoved
        self.angleMoved = 0
        return self.currentAngle
    

# myLeg = fakeLeg(0)
# myLeg.moveEmulatedServo(90)
# time.sleep(.2)
# print("Current angle: " + str(myLeg.getLocation()))
# print("")
# myLeg.moveEmulatedServo(15)
# time.sleep(.1)
# print("Current angle: " + str(myLeg.getLocation()))


p = PID(1.25, 0.8, 1.2) #Tune PID parameters here
p.setPoint(50.0)        #Move servo to 50 degrees 

myLeg = fakeLeg(-45)    #Emulated 1 dimensional leg

while True:
     pid = p.update(myLeg.getLocation())
     #print("PID output = " + str(pid))
     myLeg.moveEmulatedServo(pid)
     print("Current angle: " + str(myLeg.getLocation()))
     time.sleep(.05)
     
     

