#--------------------------------------------------
# Prototype script for control of the servos
# NOTE: remove the ### once RPIO module available
#--------------------------------------------------

###import RPIO.PWM as PWM    #defines local scope of RPIO's PWM function as PWM here

# Class definition for servo -- should be moved to proper location later
class Servo:
  """An object describing a servo with its unique ID"""
  def __init__(self, name, ID, pin):
    self._name = name
    self._ID = ID

    # Units of degrees, how much is the servo currently rotated
    self.rotation = 0

    # GPIO pin (BCM numbering) servo is controlled by
    self.pinOut = pin

  def GetID():
    return self.ID

  def GetName(self):
    return self.name

  def GetRotation():
    """Return rotation of the servo, assuming a command to rotate has been sent"""
    return self.rotation

  #--------------------------------------------------
  # Heart of the robot's control script: rotate
  # a given number of degrees on provided servo
  #--------------------------------------------------
  def Rotate( degrees ):
    """Rotate a given number of degrees."""
###    PWM.add_channel_pulse(1, self.pinOut, 0, 1000)
    return

  #------------------------------------------------------
  # Clear GPIO pin for this servo
  #------------------------------------------------------
  def StopGPIO():
###    PWM.clear_channel_gpio(1, self.pinOut)
    print("Stopped GPIO on ", flush=True)
    print(self.name, flush=True)
    print(" on pin ", flush=True)
    print(self.pinOut, flush=True)
    return

'''
TODO
- Track available DMA channels for all servos?
'''

def SetupPWM():
  #--------------------------------------------------
  # set up PWM pulse increment to 1us
  #--------------------------------------------------
###  PWM.setup(1)

  #--------------------------------------------------
  # set up DMA channel one (there are 14 available) with
  # 20ms subcycle time
  #--------------------------------------------------
###  PWM.init_channel(1, 20000)
  return


def CleanupPWM():
  # Note: must clear GPIO on each active pin first!

  #------------------------------------------------------
  # Do the final cleanup
  #------------------------------------------------------
###  PWM.clear_channel(1)
###  PWM.cleanup()
  return