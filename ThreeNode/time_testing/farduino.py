import time
from std_msgs.msg import Float64
from std_msgs.msg import Bool
import rospy

EVAP_RATE = 1.0
FLOW_RATE = 30.0


now = time.time()
count = 0
off = 0
last = time.time()

rospy.init_node("farduino")


def time_request(data):
    global now, last
    diff = data.data - now
    last = now
    now = data.data
    tick(diff)

def tick(diff):
    return diff

time_sub = rospy.Subscriber("time_raw", Float64, time_request)

while not rospy.core.is_shutdown():
    count += 1
    off = max(off, time.time() - now)

    if count > 1000:
        print(off)
        count = 0
        off = 0
    rospy.sleep(0.0001)

