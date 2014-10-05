import RPIO.PWM as PWM
import time

#--------------------------------------------------
# set up PWM pulse increment to 1us
#--------------------------------------------------
PWM.setup(1)

#--------------------------------------------------
# set up DMA channel one (there are 14 available) with
# 20ms subcycle time
#--------------------------------------------------
PWM.init_channel(1, 20000)


#--------------------------------------------------
# Select pin for servo PWM output (BCM numbering)
# See http://raspberrypi.stackexchange.com/questions/12966/what-is-the-difference-between-board-and-bcm-for-gpio-pin-numbering
#--------------------------------------------------
pin = 7

#--------------------------------------------------
# Assign PWM duty cycle http://pythonhosted.org/RPIO/pwm_py.html#examples
#--------------------------------------------------
PWM.add_channel_pulse(1, pin, 0, 1500)
time.sleep(5)
PWM.add_channel_pulse(1, pin, 0, 2000)
time.sleep(5)
PWM.add_channel_pulse(1, pin, 0, 1000)
time.sleep(5)

#------------------------------------------------------
# Clear each GPIO from the channel one by one
#------------------------------------------------------
PWM.clear_channel_gpio(1, 25)
time.sleep(1)

#------------------------------------------------------
# Do the final cleanup
#------------------------------------------------------
PWM.clear_channel(1)
PWM.cleanup()
