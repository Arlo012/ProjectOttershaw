import rospy
from std_msgs.msg import Int16

class ServoController:

	def __init__(self):
		self.speedPublisher = rospy.Publisher('ServoSpeed', Int16, queue_size = 10)
		self.anglePublisher = rospy.Publisher('ServoAngle', Int16, queue_size = 10)
		rospy.init_node('ServoController', anonymous = True)  #were we to import this as a class to a main IO handler file, this line might cause issues, in which case it should work to simply remove it

	def sendSpeed(self, speed):				#this must be published before angle
		if not rospy.is_shutdown():
			self.speedPublisher.publish(speed)

	def sendAngle(self, angle):				#this must be published after speed
		if not rospy.is_shutdown():
			self.anglePublisher.publish(angle)

if __name__ == '__main__':
	controller = ServoController()
	while True:
		try:
			speed = int(raw_input("what step size would you like the servo to sweep with? "))
			angle = int(raw_input("what angle would you like the servo to go to? "))
			controller.sendAngle(angle)
			controller.sendSpeed(speed)
		except rospy.ROSInterruptException:
			pass