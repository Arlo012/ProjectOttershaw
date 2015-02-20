class Servo:
    """Container class for a servo object"""

    def __init__(self, setID, setStartAngle, setMinAngle=0, setMaxAngle=180):
        """
        Args:
            setID (int): Unique ID correspond to Arduino hard-coded ID of this servo
            setStartAngle (int): Set initial angle of the servo for initialization (0, 180)
            setMinAngle (int, optional): Set minimum allowable servo angle (0, 180)
            setMaxAngle (int, optional): Set maximum allowable servo angle (0, 180)
        """
        self.ID = ID
        self.startAngle = setStartAngle
        self.minAngle = setMinAngle
        self.maxAngle = setMaxAngle
    
    def defineMaxAngle(self, max):
        """Safety measure: Set the maximum allowable angle of this servo"""
        self.maxAngle = max
        
    def defineMineAngle(self, min):
        """Safety measure: Set the minimum allowable angle of this servo"""
        self.minAngle = min
        
    def move(self, angle):
        """Move the servo by provided angle"""
        #TODO implement servo move
        pass
    