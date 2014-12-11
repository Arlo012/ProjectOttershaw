from serial import Serial
import collections
import queue
import time
from lib2to3.fixer_util import String
#import Singleton

class Singleton:
    """
    A non-thread-safe helper class to ease implementing singletons.
    This should be used as a decorator -- not a metaclass -- to the
    class that should be a singleton.

    The decorated class can define one `__init__` function that
    takes only the `self` argument. Other than that, there are
    no restrictions that apply to the decorated class.

    To get the singleton instance, use the `Instance` method. Trying
    to use `__call__` will result in a `TypeError` being raised.

    Limitations: The decorated class cannot be inherited from.
    http://stackoverflow.com/questions/42558/python-and-the-singleton-pattern

    """

    def __init__(self, decorated):
        self._decorated = decorated

    def Instance(self):
        """
        Returns the singleton instance. Upon its first call, it creates a
        new instance of the decorated class and calls its `__init__` method.
        On all subsequent calls, the already created instance is returned.

        """
        try:
            return self._instance
        except AttributeError:
            self._instance = self._decorated()
            return self._instance

    def __call__(self):
        raise TypeError('Singletons must be accessed through `Instance()`.')

    def __instancecheck__(self, inst):
        return isinstance(inst, self._decorated)

@Singleton
class SerialComm:
    '''Serial communication singleton - NOT THREAD SAFE'''
    receivedDataID = 0
    
    
    def __init__(self):
        #Open serial connection to the Arduino, res115200tarting it.
        #Setup parameters baud rate, and timeouts
        self.ser = Serial('/dev/ttyACM0', 9600, timeout = .5, writeTimeout = .5)
                
        #Need to sleep in order to give the Arduino tlistime to boot up
        print("Starting up the serial port -- Arduino will restart")
        time.sleep(3)
        
        #Queue of SerialCmd objects
        self.commandQueue = queue.Queue()
        print("Serial communicator created")
        
        #List of serial commands that haven't gotten a response yet
        self.sentCommands = []
        print("Successfully initialized SerialComm")
    
    def cleanup(self):
        '''Cleanup at end of serial communication'''
        self.ser.close()
        
    def read(self):
        '''
        Read in a command that was sent to the Arduino,
        check its unique ID against a list of unique IDs
        stored in the commandQueue, then decode
        '''
        
        #Read in serial value waiting on Arduino
        self.received = self.ser.readline().decode("ASCII")     #The b'....' in a string indicates it is byte coded. Here we decode it into ASCII
        #print("Received raw serial byte data: " + str(self.received))
        
        #Set the response value in the command class
        if(len(self.received) > 0):
            if(self.received[0] != "$"):
                print("Invalid received serial response")
                return ""
            
            #Parse the string into unique ID and return parameters
            else:
                self.parsedReceivedParameters = []
                self.tempParameter = ""     #For iterating through chars
                  
                self.receivedIndex = 1     #Don't decode the '$'
                #Loop through all characters in the received stream, parsing out all numbers
                while(self.received[self.receivedIndex] != "*"):
                    if(self.received[self.receivedIndex] != "!"):
                        self.tempParameter += self.received[self.receivedIndex]
                    else:
                        self.parsedReceivedParameters.append(int(self.tempParameter))    #Note: must convert to integer here
                        self.tempParameter = "" 
                    self.receivedIndex += 1
                    
                self.parsedReceivedParameters.append(int(self.tempParameter))   #TODO this is hokey -- missed this temp inside the loop
                
                self.detectedID = self.parsedReceivedParameters[0]      #We can guarantee that the first parsed parameter will be the unique ID
                
                
                #Now we know the detected ID, go checking through our list of sent commands to match this response to that ID
                for command in self.sentCommands:
                    if command.getUniqueID() == self.detectedID:
                        # We have found which command sent out that request
                        self.counter = 1    #Skip the unique ID in the array
                        self.responseOnly = []  #Response from the arduino without the unique ID
                        
                        #print("Correctly formatted response from Arduino received:")
                        while self.counter < len(self.parsedReceivedParameters):
                            self.responseOnly = self.parsedReceivedParameters[self.counter]
                            print(str(self.detectedID) + ", " + str(self.responseOnly))
                            self.counter += 1
                        command.setResponse(self.responseOnly)
                        return
                
                #Went through every command in the sent list, didn't find this one
                print("Warning: Arduino returned a unique ID not found in sent command list")
        
    def sendNextCmd(self):
        '''Send the next command in the command queue over serial line'''
        if(self.commandQueue.qsize() > 0):
            self.commandToSend = self.commandQueue.get()
            self.stringToSend = self.commandToSend.commandString
            
            #print("Sending command: " + self.stringToSend)
            self.ser.write(bytes(self.stringToSend, 'ASCII'))
            
            self.sentCommands.append(self.commandToSend)
        else:
            print("No command to send over serial comm!")
        
    def addCmd(self, serialCmd):
        '''Add serial command to command queue'''
        self.commandQueue.put(serialCmd)
    
    def flushBuffers(self):
        '''Clear the serial buffers (in and out)'''
        self.ser.flush()
    
    def flushInputBuffer(self):
        '''Clear input serial buffer'''
        self.ser.flushInput()
    
    def flushOutputBuffer(self):
        '''Clear output serial buffer'''
        self.ser.flushOutput()

          
class SerialCmd:
    '''
    Data structure of commands to be sent to Arduino over serial
    Format follows:
    Prefix: #
    Type (example): move/read
    Separator: !
    Param 1 (Cmd ID): 1
    Separator: !
    Param 2 (Servo/sensor ID): 10r
    Separator: !
    Param 3 (degrees): 90
    Suffix: *
    '''
    _responseID = 0      #Unique ID of command to be sent over the serial line 

    def __init__(self, command, passedParamList):
        self.command = str(command)
        
        #Build integer list of parameters to pass
        self.paramList = []
        for param in passedParamList:
            if type(param) is int:
                self.paramList.append(param) 
                
        #Increment the global ID number for use in this command
        SerialCmd._responseID += 1
        
        #Assign my unique ID as the current class-wide response ID value
        self._ID = SerialCmd._responseID
        
        #Build string to send out over serial line
        self.commandString = "#"
        self.commandString += self.command
        self.commandString += "!"
        self.commandString += str(self._ID) #this should include the command ID within the message
        for param in self.paramList:
            self.commandString += "!"
            self.commandString += str(param)
        self.commandString += "*"
        
        #Holder for response to serial command
        self.__response = []
        self.__responseSet = False    #Flag to detect if anyone has set this response yet
        
    def getUniqueID(self):
        return self._ID
    
    def setResponse(self, response):
        '''Sets response from Arduino, and sets received flag to true'''
        self._response = response
        self._responseSet = True
        
    def getResponse(self):
        '''
        Returns response to this command, if it has been set.
        This response is set in the SerialComm read class when this command's unique ID is detected
        '''
        if not self.__responseSet:
            print("Warning: no response has been set for this command yet.")
            return ""
        else:
            return self.__response
    
'''TEST STRUCTURE'''
#Get instance of serial communication singleton for testing
test = SerialComm.Instance()

#Create test command for moving servo #1 90 degrees
# disCommand = SerialCmd("read", [6])
# disCommand2 = SerialCmd("read", [7])
# test.addCmd(disCommand)
# test.addCmd(disCommand2)
# test.sendNextCmd()
# time.sleep(.125)
# test.sendNextCmd()

while True:
    disCommand = SerialCmd("read", [6])
    test.addCmd(disCommand)
    test.sendNextCmd()
    time.sleep(.01)      #Wait for Arduino to process
    test.read()

test.cleanup

#test.ser.write(bytes('#move!1!180*', 'ASCII'))
#test.ser.write(bytes('#Read!666!2*', 'ASCII'))