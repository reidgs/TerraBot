import rospy
import interference as interf
from std_msgs.msg import Int32

rospy.init_node("relay", anonymous = True)

r_actuator = rospy.Publisher("r_actuator", Int32, queue_size = 100)
f_sensor = rospy.Publisher("f_sensor", Int32, queue_size = 100)

def f_actuator_cb(dat):
    toPub = interf.f2r_actuator(dat.data)
    r_actuator.publish(toPub)

def r_sensor_cb(dat):
    toPub = interf.r2f_sensor(dat.data)
    f_sensor.publish(toPub)

f_actuator = rospy.Subscriber("f_actuator", Int32, f_actuator_cb)
r_sensor =  rospy.Subscriber("r_sensor", Int32, r_sensor_cb)

rospy.spin()

