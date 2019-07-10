#!/usr/bin/env python
import os
import rospy
import interference as interf
from interference import sensor_names, actuator_names, pub_types, sub_types
from std_msgs.msg import Int32,Bool,Float32
import getopt, sys
import time

verbose = False
log = False
log_files = {}
publishers = {}
subscribers = {}

def usage():
    print("relay function for Autonomous Systems")
    print("-h -l -v ")
    print("-h\thelp")
    print("-l\tlog")
    print("-v\tverbose")

def gen_log_files():
    global log_files

    prefix = time.strftime("%Y%m%d_%H%M%S_")
    os.makedirs("Log_%s" % prefix)

    for name in sensor_names + actuator_names:
        file_name = "Log_%s/%s_log.csv" % (prefix, name)
        log_files[name] = open(file_name, 'w+', 0)

def generate_publishers():
    global publishers

    for name in sensor_names:
        pub_name = name + "_output"
        publishers[name] = rospy.Publisher(
                            pub_name, pub_types[name], queue_size = 100)

    for name in actuator_names:
        pub_name = name + "_raw"
        publishers[name] = rospy.Publisher(
                            pub_name, pub_types[name], queue_size = 100)

def cb_generic(name, data):
    if (log):
        log_file = log_files[name] 
        log_file.write(str(time.time()) + ", " + str(data.data) + "\n")
        log_file.flush()
        if (verbose):
            print ("Logging %s data" % name)
    edited = interf.get_inter(name)(data.data)
    publishers[name].publish(edited)

def generate_cb(name):
    return (lambda data: cb_generic(name, data))

def generate_subscribers():
    global subscribers 
    for name in sensor_names:
        sub_name = name + "_raw"
        cb = generate_cb(name)
        subscribers[name] = rospy.Subscriber(sub_name, sub_types[name], cb)

    for name in actuator_names:
        sub_name = name + "_input"
        cb = generate_cb(name)
        subscribers[name] = rospy.Subscriber(sub_name, sub_types[name], cb)

###Start of program
try:
    opts, args = getopt.getopt(sys.argv[1:], "hlv", ["help"])

except getopt.GetoptError as err:
    # print help information and exit:
    print(err) # will print something like "option -a not recognized"
    usage()
    sys.exit(2)

for o, a in opts:
    
    if o == "-v":
        verbose = True
    elif o in ("-h", "--help"):
        usage()
        sys.exit()
    elif o in ("-l"):
        log = True
        gen_log_files()
    else:
        assert False, "unhandled option"


rospy.init_node('relay', anonymous = True)
generate_publishers()
generate_subscribers()

if (verbose):
    print("Spinning...")

rospy.spin()

