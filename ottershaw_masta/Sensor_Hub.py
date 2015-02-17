import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt16

global pub
pub = rospy.Publisher('Control', String, queue_size=10)

def sensor_shelf():
     rospy.init_node('sensor_shelf', anonymous=True)
     
     rospy.Subscriber("sonar", UInt16, sonar)
     rospy.Subscriber("gyro", UInt16, gyro)
     rospy.Subscriber("pressure1", UInt16, pressure1)
     rospy.Subscriber("pressure2", UInt16, pressure2)
     rospy.Subscriber("Control", String, controller)
     
     rospy.spin()

def sonar(data):
    global sonar_data
    sonar_data = data.data
    
def gyro(data):
    global gyro_data
    gyro_data = data.data
    
def pressure1(data):
	global pressure1_data
	pressure1_data = data.data
    
def pressure2(data):
	global pressure2_data
	pressure2_data = data.data
    
def controller(data):
    if type =="sonar":
        pub.publish(sonar_data)
    elif type == "gyro":
        pub.publish(gyro_data)
    else:
        respose ="Its a quarter passed your MoM"
        pub.publish(resposese)
        
if __name__ == '__main__':
  	listener()
  