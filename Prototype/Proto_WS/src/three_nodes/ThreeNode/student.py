import rospy
from std_msgs.msg import Int32

rospy.init_node("student", anonymous = True)

wpump_pub = rospy.Publisher("wpump_raw", Int32, queue_size = 100)
apump_pub = rospy.Publisher("apump_raw", Int32, queue_size = 100)
led_pub = rospy.Publisher("led_raw", Int32, queue_size = 100)

