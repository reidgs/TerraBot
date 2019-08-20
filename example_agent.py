#!/usr/bin/env python

#mock file of student
import rospy
import subprocess
from std_msgs.msg import Float32, Int32, Bool, String, Float32MultiArray, Int32MultiArray
import time

light = 0
led_level = 0
hum  = 0
w_level = 3


rospy.set_param("use_sim_time", True)

rospy.init_node("student", anonymous = True)

wpump_pub = rospy.Publisher("wpump_input", Bool, latch = True, queue_size = 1)
led_pub = rospy.Publisher("led_input", Int32, latch = True, queue_size = 1)
fan_pub = rospy.Publisher("fan_input", Bool, latch = True, queue_size = 1)

ping_pub = rospy.Publisher("ping", Bool, latch = True, queue_size = 1)

def humid_reaction(data):
    global hum
    hum = data.data[0]

def temp_reaction(data):
    True

def light_reaction(data):
    global light
    light = data.data[0]

def level_reaction(data):
    global w_level
    w_level = data.data

def cam_reaction(data):
    print ("picture taken\t" + data.data)


temp_sensor = rospy.Subscriber("temp_output", Int32MultiArray, temp_reaction)
humid_sensor = rospy.Subscriber("humid_output", Int32MultiArray, humid_reaction)
light_sensor = rospy.Subscriber("light_output", Int32MultiArray, light_reaction)
level_sensor = rospy.Subscriber("level_output", Float32, level_reaction)

time_now = rospy.get_time()
light_time = rospy.get_time() - 13 * 3600
cam_time = 0

while not rospy.core.is_shutdown():
    time_now = rospy.get_time()

    if time_now % 300 == 0:
        ping_pub.publish(True)

    if time_now - light_time > 12 * 3600:
        led_level ^= 255
        light_time = time_now
        led_pub.publish(led_level)

    if time_now - cam_time > 3600:
        time_stamp = "Photos/" + str(rospy.get_time()) + ".jpg"
        subprocess.call("raspistill -n -md 2 -awb off -awbg 1,1 -ss 30000 -o %s" % time_stamp, shell = True)
        cam_time = time_now

    #fan_pub.publish(True if hum > 70 else False)
    wpump_pub.publish(True if w_level < 100 else False)
    rospy.sleep(1)


