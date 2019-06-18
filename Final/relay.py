import rospy
import interferance as interf
from std_msgs.msg import Int32

rospy.init_node("relay", anonymous = True)

#publishing sensor data to the student
humid_pub = rospy.Publisher("humid_fine", Int32, queue_size = 100)
temp_pub = rospy.Publisher("temp_fine", Int32, queue_size = 100)
light_pub = rospy.Publisher("light_fine", Int32, queue_size = 100)
level_pub = rospy.Publisher("level_fine", Int32, queue_size = 100)
tds_pub = rospy.Publisher("tds_fine", Int32, queue_size = 100)

def humid_p(data):
    edited = interf.humid_inter(data.data)
    humid_pub.publish(edited)

def temp_p(data):
    edited = interf.temp_inter(data.data)
    temp_pub.publish(edited)

def light_p(data):
    edited = interf.light_inter(data.data)
    light_pub.publish(edited)

def level_p(data):
    edited = interf.level_inter(data.data)
    level_pub.publish(edited)

def tds_p(data):
    edited = interf.tds_inter(data.data)
    tds_pub.publish(edited)

humid_sensor = rospy.Subscriber("humid_raw", Int32, humid_p)
temp_sensor = rospy.Subscriber("temp_raw", Int32, temp_p)
light_sensor = rospy.Subscriber("light_raw", Int32, light_p)
level_sensor = rospy.Subscriber("level_raw", Int32, level_p)
tds_sensor = rospy.Subscriber("tds_raw", Int32, tds_p)

#publishing actuator data to the arduino

wpump_pub = rospy.Publisher("wpump_fine", Int32, queue_size = 100)
apump_pub = rospy.Publisher("apump_fine", Int32, queue_size = 100)
led_pub = rospy.Publisher("led_fine", Int32, queue_size = 100)

def wpump_p(data):
    edited = interf.wpump_inter(data.data)
    wpump_pub.publish(edited)

def apump_p(data):
    edited = interf.apump_inter(data.data)
    apump_pub.publish(edited)

def led_p(data):
    edited = interf.led_inter(data.data)
    led_pub.publish(edited)

wpump_input = rospy.Subscriber("wpump_raw", Int32, wpump_p)
apump_input = rospy.Subscriber("apump_raw", Int32, apump_p)
led_input = rospy.Subscriber("led_raw", Int32, led_p)

