from std_msgs.msg import Float64, Bool
import rospy
import time

rospy.init_node("relay_tester")

time_pub = rospy.Publisher("time_raw", Float64, queue_size = 100)

def get_time():
    time_pub.publish(time.time())

while not rospy.core.is_shutdown():
    get_time()
    rospy.sleep(0.001)

