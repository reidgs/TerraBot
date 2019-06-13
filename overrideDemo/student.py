import rospy
from std_msgs.msg import Int32

rospy.init_node("student", anonymous = True)

actuator = rospy.Publisher("f_actuator", Int32, queue_size = 100)

def sensor_cb(dat):
    actuator.publish(dat.data)

sensor =  rospy.Subscriber("f_sensor", Int32, sensor_cb)

rospy.spin()

