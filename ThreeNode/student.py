#!/usr/bin/env python

#mock file of student
import rospy
import subprocess
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import String
import time

light = 0
led_level = 0
hum  = 0
w_level = 3

def get_time():
    return time.time()

light_time = 0
cam_time = 0

rospy.init_node("student", anonymous = True)

wpump_pub = rospy.Publisher("wpump_input", Bool, latch = True, queue_size = 100)
npump_pub = rospy.Publisher("npump_input", Bool, latch = True, queue_size = 100)
apump_pub = rospy.Publisher("apump_input", Bool, latch = True, queue_size = 100)
led_pub = rospy.Publisher("led_input", Int32, latch = True, queue_size = 100)
fan_pub = rospy.Publisher("fan_input", Bool, latch = True, queue_size = 100)

def humid_reaction(data):
    global hum
    hum = data.data

def temp_reaction(data):
    True

def light_reaction(data):
    global light
    light = data.data

def level_reaction(data):
    global w_level
    w_level = data.data

def tds_reaction(data):
    True

def cam_reaction(data):
    print ("picture taken\t" + data.data)

temp_sensor = rospy.Subscriber("temp_output", Int32, temp_reaction)
humid_sensor = rospy.Subscriber("humid_output", Int32, humid_reaction)
light_sensor = rospy.Subscriber("light_output", Int32, light_reaction)
level_sensor = rospy.Subscriber("level_output", Int32, level_reaction)
tds_sensor = rospy.Subscriber("tds_output", Int32, tds_reaction)

while not rospy.core.is_shutdown():
    time_now = get_time()

    if time_now - light_time > 6 * 3600:
        led_level ^= 255
        light_time = time_now
        led_pub.publish(led_level)

    if time_now - cam_time > 1800:
        time_stamp = "Photos/" + str(time.time()) + ".jpg"
        subprocess.call("raspistill -n -md 2 -awb off -awbg 1,1 -ss 30000 -o %s" % time_stamp, shell = True)
        cam_time = time_now

    fan_pub.publish(True if hum > 70 else False)
    wpump_pub.publish(True if w_level == 0 else False)
    rospy.rostime.wallsleep(1)


