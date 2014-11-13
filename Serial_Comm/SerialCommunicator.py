import serial
import collections
import queue
import time
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
        self.ser = serial.Serial('/dev/ttyACM1', 9600, timeout = 0.5, writeTimeout = 0.5)
        
        #Queue of SerialCmd objects
        self.commandQueue = queue.Queue()
        print("Serial communicator created")
        
        #List of responses from serial comm
        self.receivedList = []
        print("Successfully initialized SerialComm")
    
    def cleanup(self):
        '''Cleanup at end of serial communication'''
        self.ser.close()
        
    def read(self, command):
        '''
        Read in a command that was sent to the Arduino,
        and then get the response and add to received list
        on top of the existing command.
        '''
        
        #Read in serial value waiting on Arduino
        self.received = self.ser.readline()
        #print("Received raw serial byte data: " + str(self.received))
        
        #Set the response value in the command class
        command.response = self.received
        
        #Append the whole command to the received list (so command sent data can be used)
        self.receivedList.append(command)
    
    def sendNextCmd(self):
        '''Send the next command in the command queue over serial line'''
        if(self.commandQueue.qsize() > 0):
            self.unparsedCommand = self.commandQueue.get()
            self.stringToSend = str(self.unparsedCommand.command)
            self.stringToSend += ","
            self.stringToSend += str(self.unparsedCommand.paramList)
            
            print("Sending command: " + self.stringToSend)
            self.ser.write(bytes(self.stringToSend, 'UTF-8'))
            
            #Read back the response from the arduino, append to the command
            self.read(self.unparsedCommand)
        else:
            print("No command to send over serial comm!")
        
    def addCmd(self, serialCmd):
        '''Add serial command to command queue'''
        self.commandQueue.put(serialCmd)
        
    def getResponse(self, ID):
        '''Get the response to the command based on unique ID'''
        for item in self.receivedList:
            if item.ID == ID:
                self.tempItem = item.response
                self.receivedList.remove(item)
                return self.tempItem
            
        print("Failed to find response item for provided ID: " + str(ID))
        return "[No response]"
    
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
    '''Data structure of commands to be sent to Arduino over serial'''
    responseID = 0
    
    def __init__(self, command, paramList):
        self.command = str(command)
        
        #Comma separated value of parameters to send
        self.paramList = str(paramList)
        
        self.ID = self.responseID
        self.responseID += 1
        
        #Holder for response to serial command
        self.response = []        

'''TEST STRUCTURE'''
test = SerialComm.Instance()
#command = SerialCmd("MoveServo", "90,")
test.ser.write(bytes("TEST123", 'ASCII'))
#test.sendNextCmd()
#print("Values in waiting on receive buffer: " + str(test.ser.inWaiting()))
#test.flushOutputBuffer()
#time.sleep(1)
test.cleanup




