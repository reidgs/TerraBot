#!/usr/bin/env python
import os, sys
import subprocess as sp
import signal
import rospy
import interference as interf
from topic_def import *
from std_msgs.msg import Int32,Bool,Float32,String
from rosgraph_msgs.msg import Clock
import argparse
import time
import grader

### Default values for the optional variables
verbose = False
grade = False
log = False
simulate = False
still_running = True

global clock_time

### lists which will be populated based on the topic_def.py
log_files = {}
publishers = {}
subscribers = {}

actuator_vars = {}

def gen_log_files():
    global log_files

    prefix = time.strftime("%Y%m%d_%H%M%S") + ("_sim" if simulate else "")
    os.makedirs("Log/Log_%s" % prefix)

    for name in sensor_names + actuator_names:
        file_name = "Log/Log_%s/%s_log.csv" % (prefix, name)
        log_files[name] = open(file_name, 'w+', 0)

def log_print(string):
    print("%s%s"%(time.strftime("[%Y%m%d %H:%M:%S]: "),string))

def generate_publishers():
    global publishers

    for name in sensor_names:
        pub_name = name + "_output"
        publishers[name] = rospy.Publisher(
                            pub_name, to_stu[name],
                            latch = True, queue_size = 100)

    for name in actuator_names:
        pub_name = name + "_raw"
        publishers[name] = rospy.Publisher(
                            pub_name, to_ard[name],
                            latch = True, queue_size = 100)

def cb_generic(name, data):
    global clock_time
    if (log):
        log_file = log_files[name]
        log_file.write(str(clock_time.to_sec()) + ", " + str(data.data) + "\n")
        log_file.flush()
        if (verbose):
            log_print ("Logging %s data" % name)
    if grade:
        actuator_vars[name] = data.data

    edited = interf.get_inter(name)(data.data)
    publishers[name].publish(edited)

def generate_cb(name):
    return (lambda data: cb_generic(name, data))

def generate_subscribers():
    global subscribers
    for name in sensor_names:
        sub_name = name + "_raw"
        cb = generate_cb(name)
        subscribers[name] = rospy.Subscriber(sub_name, from_ard[name], cb)

    for name in actuator_names:
        sub_name = name + "_input"
        cb = generate_cb(name)
        subscribers[name] = rospy.Subscriber(sub_name, from_stu[name], cb)

###Start of program
parser = argparse.ArgumentParser(description = "relay parser for Autonomous Systems")
parser.add_argument('-v', '--verbose', action = 'store_true')
parser.add_argument('-l', '--log', action = 'store_true')
parser.add_argument('-s', '--simulate', action = 'store_true')
parser.add_argument('-m', '--mode', default = "serial")
parser.add_argument('-g', '--grade', action = 'store_true')
parser.add_argument('--speedup', default = 1, type = float)

args = parser.parse_args()

verbose = args.verbose
log = args.log
simulate = args.mode == "sim"
speedup = args.speedup
grade = args.mode == "grade"

if log:
    gen_log_files()

### Open logs for roscore and relay
core_log = open("Log/roscore.log", "a+", 0)
relay_log = open("Log/relay.log", "a+", 0)

### Start up roscore, redirecting output to logging files
core_p = sp.Popen("roscore", stdout = core_log, stderr = core_log)

### Begin relay node
rospy.init_node('relay', anonymous = True)
rospy.set_param("use_sim_time", True)

generate_publishers()
generate_subscribers()

clock_pub = rospy.Publisher("clock", Clock, latch = True, queue_size = 1000)

### Health Ping callback function
### Records the most recent ping
def ping_cb(data):
    global last_ping, simulate, clock_time
    last_ping = clock_time if simulate else rospy.get_rostime()

ping_sub = rospy.Subscriber('ping', Bool, ping_cb)

### Spawn subprocesses

### Simulator starts up Student and Farduino
#TODO add baseline functionality
student_log = open("Log/student.log", "a+", 0)

if simulate:
    sim_log = open("Log/simulator.log", "a+", 0)
    ### Initiates the Simulator and redirects output
    sim_p = sp.Popen(["python", "farduino.py"],
        stdout = sim_log, stderr = sim_log)
    ### Initiates the Student file and redirects output
    student_p = sp.Popen(["python", "student.py"],
        stdout = student_log, stderr = student_log)

### Running the real arduino proccess
elif not grade:
    serial_log = open("Log/rosserial.log", "a+", 0)
    ### Initiates the Arduino and redirects output
    serial_p = sp.Popen(["rosrun", "rosserial_arduino",
        "serial_node.py", "/dev/ttyACM0"],
        stdout = serial_log, stderr = serial_log)
    ### Initiates the Student file and redirects output
    student_p = sp.Popen(["python", "student.py"],
        stdout = student_log, stderr = student_log)



if (verbose):
    log_print("Spinning...")


### Loop for the entire system, should only ever break if grading
while still_running:
    ### TODO add grading functionality
    if grade:
        grader.open_trace("trace.txt")

        student_p = sp.Popen(["python", "student.py"],
            stdout = student_log, stderr = student_log)

        sim_log = open("Log/simulator.log", "a+", 0)
        ### Initiates the Simulator and redirects output
        sim_p = sp.Popen(["python", "farduino.py", grader.bfile],
            stdout = sim_log, stderr = sim_log)


    ### We must begin simulated time and health ping
    clock_time = rospy.Time(0)
    last_ping = rospy.Time(0)
    ### Loop for a single iteration of a grading scheme(infinite if not grading)
    while not rospy.core.is_shutdown():
        ### If not simulating get real time
        #print("spin")
        if not simulate and not grade:
            clock_time = rospy.get_rostime()
        clock_pub.publish(clock_time)
        ### TODO add grading functionality
        if grade:
            task = 'unaccomplished'
            cur_cmd = grader.run_command(clock_time)
            start_time = clock_time
            if grader.finished:
                print("break")
                break
            instr,actu,state,dur = cmd[0],cmd[1],cmd[2],cmd[3]
            if instr == 'WAIT':
                while clock_time <= start_time + dur:
                    if actuator_vars[actu] == state:
                        task = 'accomplished'
                        break
            elif instr == 'ENSURE':
                while clock_time <= start_time + dur:
                    if actuator_vars[actu] == state:
                        task = 'accomplished'
                    else:
                        task = 'unaccmplished'
                        break
            print(task)



        ### TODO Fix the ping so that it actually works
        last_ping = clock_time
        if  clock_time.to_sec() - last_ping.to_sec() > 3600:
            log_print("no ping since %f, terminating..."%last_ping.to_sec())
            student_p.terminate()

        if (student_p.poll() != None):
            log_print("student restarting...")
            student_p = sp.Popen(["python", "student.py"],
                    stdout = student_log, stderr = student_log)

        clock_time += rospy.Duration(0.1)
        rospy.sleep(0.1/speedup)
    #sim_p.terminate()
    student_p.terminate()
    reload(grader)
    break;

core_p.terminate()

