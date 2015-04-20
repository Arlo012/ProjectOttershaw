import rospy
from std_msgs.msg import String

class TesterTalker:

	def __init__(self):
		self.publisher = rospy.Publisher('test', String, queue_size = 10)		#publishing to the chatter topic, in the form of a string, and only lets 10 publications sit in the queue before removing old ones.  This is created as a publishing object.
		rospy.init_node('test', anonymous = True)		#initialized the node with the name that it will be addressed as by the ROS Master communicator

	def sendMessage(self, message):
		if not rospy.is_shutdown():		#checks to see if the program should be exiting for something like a Ctrl-C command
			rospy.loginfo(message)		#prints the message to the screen, the ROS console, and the message gets put in the nodes log file
			self.publisher.publish(message)	#takes the input string, and then publishes it to the 'test' message board from our publisher object

if __name__ == '__main__':		#this code will only run if you run this code as an independent program, will not run if it is imported from somewhere else that wants to use the classes here
	tester = TesterTalker()		#creates a TesterTalker object
	tester2 = TesterTalker()
	while True:	
		try:		#checks to make sure that the node is still active and that the process hasn't been shutdown by a Ctrl-C
			message = raw_input("type a message please: ")		#prompts for a message
			tester.sendMessage(message)	#runs the send message method from our tester object
			tester2.sendMessage(message + " (this was the message from tester2)")  #testing for multiple objects
			tester.sendMessage(message + " tester1")	
		except rospy.ROSInterruptException:	#corresponds to the try
			pass