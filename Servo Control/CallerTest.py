# Caller test (calling basicControl.py functions)
import basicControl as control

x = control.Servo("Servo1", 0, 7)
print( x.GetName() )