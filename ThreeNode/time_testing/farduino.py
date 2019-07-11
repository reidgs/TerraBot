import time
from std_msgs.msg import Float64
from std_msgs.msg import Bool
import rospy

rospy.init_node("farduino")

time_pub = rospy.Publisher("time_raw", Float64, queue_size = 100)

def time_request(data):
    time_pub.publish(time.time())

time_sub = rospy.Subscriber("time_req", Bool, time_request)

rospy.spin()
