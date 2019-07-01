#!/usr/bin/env python
import rospy
import interference as interf
from std_msgs.msg import Int32
from std_msgs.msg import Bool
import getopt, sys
import time

def usage():
    print("relay function for Autonomous Systems")
    print("-h -l -v [file]")
    print("-h\thelp")
    print("-l\tlog")
    print("DO NOT INCLUE .txt or any file extensions in the log file")
    print("-v\tverbose")

try:
    opts, args = getopt.getopt(sys.argv[1:], "hl:v", ["help"])
except getopt.GetoptError as err:
    # print help information and exit:
    print(err) # will print something like "option -a not recognized"
    usage()
    sys.exit(2)
verbose = False
log = False
log_file = None
for o, a in opts:
    if o == "-v":
        verbose = True
    elif o in ("-h", "--help"):
        usage()
        sys.exit()
    elif o in ("-l"):
        log = True
        log_file = a
        humid_file = open(a + "_humid_log.txt", a, 0)
    else:
        assert False, "unhandled option"

rospy.init_node('relay', anonymous = True)

#publishing sensor data to the student
humid_pub = rospy.Publisher("humid_output", Int32, queue_size = 100)
temp_pub = rospy.Publisher("temp_output", Int32, queue_size = 100)
light_pub = rospy.Publisher("light_output", Int32, queue_size = 100)
level_pub = rospy.Publisher("level_output", Int32, queue_size = 100)
tds_pub = rospy.Publisher("tds_output", Int32, queue_size = 100)

def humid_p(data):
    if (log and verbose):
        print("logging humidity from arduino")
        humid_file.write(data.data + ", " + time.time())
    edited = interf.humid_inter(data.data)
    humid_pub.publish(edited)

def temp_p(data):
    if (log and verbose):
        print("logging temperature from arduino")
    edited = interf.temp_inter(data.data)
    temp_pub.publish(edited)

def light_p(data):
    if (log and verbose):
        print("logging light from arduino")
    edited = interf.light_inter(data.data)
    light_pub.publish(edited)

def level_p(data):
    if (log and verbose):
        print("logging water level from arduino")
    edited = interf.level_inter(data.data)
    level_pub.publish(edited)

def tds_p(data):
    if (log and verbose):
        print("logging tds from arduino")
    edited = interf.tds_inter(data.data)
    tds_pub.publish(edited)

humid_sensor = rospy.Subscriber("humid_raw", Int32, humid_p)
temp_sensor = rospy.Subscriber("temp_raw", Int32, temp_p)
light_sensor = rospy.Subscriber("light_raw", Int32, light_p)
level_sensor = rospy.Subscriber("level_raw", Int32, level_p)
tds_sensor = rospy.Subscriber("tds_raw", Int32, tds_p)

#publishing actuator data to the arduino

led_pub = rospy.Publisher("led_raw", Int32, queue_size = 100)
wpump_pub = rospy.Publisher("wpump_raw", Bool, queue_size = 100)
npump_pub = rospy.Publisher("npump_raw", Bool, queue_size = 100)
apump_pub = rospy.Publisher("apump_raw", Bool, queue_size = 100)

def led_p(data):
    if(verbose and log):
        print("logging led input from student")
    edited = interf.led_inter(data.data)
    led_pub.publish(edited)

def wpump_p(data):
    if(verbose and log):
        print("logging water pump input from student")
    edited = interf.wpump_inter(data.data)
    wpump_pub.publish(edited)

def npump_p(data):
    if(verbose and log):
        print("logging nutrient pump input from student")
    edited = interf.npump_inter(data.data)
    npump_pub.publish(edited)

def apump_p(data):
    if(verbose and log):
        print("logging air pump input from student")
    edited = interf.apump_inter(data.data)
    apump_pub.publish(edited)

led_input = rospy.Subscriber("led_input", Int32, led_p)
wpump_input = rospy.Subscriber("wpump_input", Bool, wpump_p)
npump_input = rospy.Subscriber("npump_input", Bool, npump_p)
apump_input = rospy.Subscriber("apump_input", Bool, apump_p)

if (verbose):
    print("Spinning...")
rospy.spin()
