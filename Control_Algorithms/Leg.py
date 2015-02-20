class Leg:
    "A 3-jointed leg object consisting of servos"
    
    def __init__(self, bodyServo, shoulderServo, kneeServo):
        """
        Args:
            bodyServo (Servo): Servo object for moving forward-back
            shoulderServo (Servo): Servo object for moving leg up-down
            kneeServo (Servo): Servo object for moving lower leg up-down
        """
        self.bodyServo = bodyServo
        self.shoulderServo = shoulderServo
        self.kneeServo = kneeServo
        
    def move(self):
        #TODO: define move parameters
        pass
        
    