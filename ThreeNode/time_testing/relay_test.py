from std_msgs.msg import Float64, Bool
import rospy
import time

rospy.init_node("relay_tester")

time_now = 0.0

time_req_pub = rospy.Publisher("time_req", Bool, queue_size = 100)

def set_time(data):
    global time_now
    time_now = data.data

time_req_sub = rospy.Subscriber("time_raw", Float64, set_time)

def get_time():
    time_req_pub.publish(True)
    return time_now

while not rospy.core.is_shutdown():
    print ("real time:\t %.16f" %time.time())
    print ("topic time:\t %.16f" %get_time())
    print ("-----------------")
    rospy.rostime.wallsleep(0.01)

