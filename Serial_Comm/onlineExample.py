from serial import Serial
import time
 
 #Open serial connection to the Arduino, restarting it.
 #Setup parameters baud rate, and timeouts
ser = Serial('/dev/ttyACM0', 
            115200, timeout = .5, 
             writeTimeout = .5)

#Need to sleep in order to give the Arduino time to boot up
time.sleep(3)

ser.write(bytes('1', 'ASCII'))
x = ser.readline()
print(str(x))
