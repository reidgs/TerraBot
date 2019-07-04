#!/usr/bin/env python
import rospy
import interference as interf
from std_msgs.msg import Int32
from std_msgs.msg import Bool
import getopt, sys
import time

def usage():
    print("relay function for Autonomous Systems")
    print("-h -l -v ")
    print("-h\thelp")
    print("-l\tlog")
    print("-v\tverbose")

try:
    opts, args = getopt.getopt(sys.argv[1:], "hlv", ["help"])
except getopt.GetoptError as err:
    # print help information and exit:
    print(err) # will print something like "option -a not recognized"
    usage()
    sys.exit(2)
verbose = False
log = False
for o, a in opts:
    if o == "-v":
        verbose = True
    elif o in ("-h", "--help"):
        usage()
        sys.exit()
    elif o in ("-l"):
        log = True
        humid_file = open("Log/humid_log.csv", 'a', 0)
        temp_file = open("Log/temp_log.csv", 'a', 0)
        light_file = open("Log/light_log.csv", 'a', 0)
        level_file = open("Log/level_log.csv", 'a', 0)
        tds_file = open("Log/tds_log.csv", 'a', 0)
        cur_file = open("Log/cur_log.csv", 'a', 0)
        led_file = open("Log/led_log.csv", 'a', 0)
        wpump_file = open("Log/wpump_log.csv", 'a', 0)
        npump_file = open("Log/npump_log.csv", 'a', 0)
        apump_file = open("Log/apump_log.csv", 'a', 0)
        fan_file = open("Log/fan_log.csv", 'a', 0)
    else:
        assert False, "unhandled option"

rospy.init_node('relay', anonymous = True)

#publishing sensor data to the student
humid_pub = rospy.Publisher("humid_output", Int32, queue_size = 100)
temp_pub = rospy.Publisher("temp_output", Int32, queue_size = 100)
light_pub = rospy.Publisher("light_output", Int32, queue_size = 100)
level_pub = rospy.Publisher("level_output", Int32, queue_size = 100)
tds_pub = rospy.Publisher("tds_output", Int32, queue_size = 100)
cur_pub = rospy.Publisher("cur_output", Int32, queue_size = 100)

def humid_p(data):
    if (log):
        humid_file.write(str(time.time()) + ", " + str(data.data) + "\n")
        flush(humid_file)
        if (verbose):
            print ("Logging humidity data")
    edited = interf.humid_inter(data.data)
    humid_pub.publish(edited)

def temp_p(data):
    if (log):
        temp_file.write(str(time.time()) + ", " + str(data.data) + "\n")
        flush(temp_file)
        if (verbose):
            print ("Logging Tempurature data")
    edited = interf.temp_inter(data.data)
    temp_pub.publish(edited)

def light_p(data):
    if (log):
        light_file.write(str(time.time()) + ", " + str(data.data) + "\n")
        flush(light_file)
        if (verbose):
            print ("Logging light data")
    edited = interf.light_inter(data.data)
    light_pub.publish(edited)

def level_p(data):
    if (log):
        level_file.write(str(time.time()) + ", " + str(data.data) + "\n")
        flush(level_file)
        if (verbose):
            print ("Logging water level data")
    edited = interf.level_inter(data.data)
    level_pub.publish(edited)

def tds_p(data):
    if (log):
        tds_file.write(str(time.time()) + ", " + str(data.data) + "\n")
        flush(tds_file)
        if (verbose):
            print ("Logging tds data")
    edited = interf.tds_inter(data.data)
    tds_pub.publish(edited)

def cur_p(data):
    if (log):
        cur_file.write(str(time.time()) + ", " + str(data.data) + "\n")
        flush(cur_file)
        if (verbose):
            print ("Logging cur data")
    edited = interf.cur_inter(data.data)
    cur_pub.publish(edited)

humid_sensor = rospy.Subscriber("humid_raw", Int32, humid_p)
temp_sensor = rospy.Subscriber("temp_raw", Int32, temp_p)
light_sensor = rospy.Subscriber("light_raw", Int32, light_p)
level_sensor = rospy.Subscriber("level_raw", Int32, level_p)
tds_sensor = rospy.Subscriber("tds_raw", Int32, tds_p)
cur_sensor = rospy.Subscriber("cur_raw", Int32, cur_p)

#publishing actuator data to the arduino

led_pub = rospy.Publisher("led_raw", Int32, queue_size = 100)
wpump_pub = rospy.Publisher("wpump_raw", Bool, queue_size = 100)
npump_pub = rospy.Publisher("npump_raw", Bool, queue_size = 100)
apump_pub = rospy.Publisher("apump_raw", Bool, queue_size = 100)
fan_pub = rospy.Publisher("fan_raw", Bool, queue_size = 100)

def led_p(data):
    if (log):
        led_file.write(str(time.time()) + ", " + str(data.data) + "\n")
        flush(led_file)
        if (verbose):
            print ("Logging LED data")
    edited = interf.led_inter(data.data)
    led_pub.publish(edited)

def wpump_p(data):
    if (log):
        wpump_file.write(str(time.time()) + ", " + str(data.data) + "\n")
        flush(wpump_file)
        if (verbose):
            print ("Logging water pump data")
    edited = interf.wpump_inter(data.data)
    wpump_pub.publish(edited)

def npump_p(data):
    if (log):
        npump_file.write(str(time.time()) + ", " + str(data.data) + "\n")
        flush(npump_file)
        if (verbose):
            print ("Logging nutrient pump data")
    edited = interf.npump_inter(data.data)
    npump_pub.publish(edited)

def apump_p(data):
    if (log):
        apump_file.write(str(time.time()) + ", " + str(data.data) + "\n")
        flush(apump_file)
        if (verbose):
            print ("Logging air pump data")
    edited = interf.apump_inter(data.data)
    apump_pub.publish(edited)

def fan_p(data):
    if (log):
        fan_file.write(str(time.time()) + ", " + str(data.data) + "\n")
        flush(fan_file)
        if (verbose):
            print ("Logging fan data")
    edited = interf.fan_inter(data.data)
    fan_pub.publish(edited)

led_input = rospy.Subscriber("led_input", Int32, led_p)
wpump_input = rospy.Subscriber("wpump_input", Bool, wpump_p)
npump_input = rospy.Subscriber("npump_input", Bool, npump_p)
apump_input = rospy.Subscriber("apump_input", Bool, apump_p)
fan_input = rospy.Subscriber("fan_input", Bool, fan_p)

if (verbose):
    print("Spinning...")
rospy.spin()
