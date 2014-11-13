#!/usr/bin/python
# -*- coding: utf-8 -*-
 
from serial import Serial
import time
 
ser = Serial('/dev/ttyACM0', 
            115200, timeout = .5, 
             writeTimeout = .5)
time.sleep(3)

ser.write(bytes('1', 'ASCII'))
x = ser.readline()
print(str(x))
