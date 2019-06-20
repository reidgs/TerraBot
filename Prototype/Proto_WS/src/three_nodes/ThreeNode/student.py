#!/usr/bin/env python

#mock file of student
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Bool

rospy.init_node("student", anonymous = True)

wpump_pub = rospy.Publisher("wpump_input", Bool, queue_size = 100)
npump_pub = rospy.Publisher("npump_input", Bool, queue_size = 100)
apump_pub = rospy.Publisher("apump_input", Bool, queue_size = 100)
led_pub = rospy.Publisher("led_input", Int32, queue_size = 100)

def humid_reaction(data):
    True

def temp_reaction(data):
    True
	
def light_reaction(data):
    True

def level_reaction(data):
    True

def tds_reaction(data):
    True

humid_sensor = rospy.Subscriber("humid_output", Int32, humid_reaction)
temp_sensor = rospy.Subscriber("temp_output", Int32, temp_reaction)
light_sensor = rospy.Subscriber("light_output", Int32, light_reaction)
level_sensor = rospy.Subscriber("level_output", Int32, level_reaction)
tds_sensor = rospy.Subscriber("tds_output", Int32, tds_reaction)

rospy.spin()
