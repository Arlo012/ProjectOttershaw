#----------------------------------------
# Test of simple IO. Compares print() and
# stdout() functions, uses OR logic
#----------------------------------------

import time
import sys  #Use this instead of print for more complex tasks where you need more control

#Test of python terminal IO

username = input("Hello, this is your Raspberry Pi. Please provide your name or be annihilated.\n")
if(username == "no" or username == "No" or username == "NO"):
  #Standard print line, ends with a return
  print("No problem, hu-man!")

  #Time function, puts processor on hold for 2 seconds
  time.sleep(2)

  #Need print parameters, see here: http://stackoverflow.com/questions/493386/how-to-print-in-python-without-newline-or-space
  print("Missile launch in:", flush=True)
  print("3", end = "", flush=True)
  time.sleep(.44)
  print(".", end = "", flush=True)
  time.sleep(.44)
  print(".", end = "", flush=True)
  time.sleep(.44)
  print(".")

  print("2", end = "", flush=True)
  time.sleep(.44)
  print(".", end = "", flush=True)
  time.sleep(.44)
  print(".", end = "", flush=True)
  time.sleep(.44)
  print(".")

  #Alternatively, here is system out version of the same task
  sys.stdout.write('1')
  sys.stdout.flush()    #Force the output of the standard buffer (don't need unless sleeping right after)
  time.sleep(.44)
  sys.stdout.write('.')
  sys.stdout.flush()
  time.sleep(.44)
  sys.stdout.write('.')
  sys.stdout.flush()
  time.sleep(.44)
  sys.stdout.write('.\n')

  print("Goodbye :)")

else:
  #Using print here because it is simple, and I want a return at the end of the line
  print ("It is my pleasure to meet you " + username + ", have a nice day")
