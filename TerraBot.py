#!/usr/bin/env python
import os, sys, glob
import select
import subprocess as sp
import signal
import rospy
from lib import interference as interf
from lib import topic_def as tdef
from std_msgs.msg import Int32,Bool,Float32,String,Int32MultiArray,Float32MultiArray
import rosgraph
import argparse
import time
from lib import grader

### Default values for the optional variables
verbose = False
grade = False
log = False
simulate = False
still_running = True
run_agent = True
#tick_interval = 0.01
tick_interval = 0.5
clock_time = rospy.Time(0)

#lists which will be populated based on the topic_def.py
log_files = {}
publishers = {}
subscribers = {}
grader_vars = {}

def gen_log_files():
    global log_files

    prefix = time.strftime("%Y%m%d_%H%M%S") + ("_sim" if simulate else "")
    os.makedirs("Log/Log_%s" % prefix)

    for name in tdef.sensor_names + tdef.actuator_names:
        file_name = "Log/Log_%s/%s_log.csv" % (prefix, name)
        log_files[name] = open(file_name, 'w+', 0)

def log_print(string):
    print("%s%s"%(time.strftime("[%Y%m%d %H:%M:%S]: "),string))

def generate_publishers():
    global publishers

    for name in tdef.sensor_names:
        pub_name = name + "_output"
        publishers[name] = rospy.Publisher(
                            pub_name, tdef.sensor_types[name],
                            latch = True, queue_size = 1)

    for name in tdef.actuator_names:
        pub_name = name + "_raw"
        publishers[name] = rospy.Publisher(
                            pub_name, tdef.actuator_types[name],
                            latch = True, queue_size = 1)

def cb_generic(name, data):
    global clock_time, grader_vars
    interf_func = interf.get_inter(name, clock_time)
    original = data.data
    edited = data
    #redundant sensors
    if (name in tdef.sensor_names) and (name != 'level'):
        edited.data = [interf_func[0](name, data.data[0]), \
                       interf_func[1](name, data.data[1])]
    else:
        edited.data = interf_func(name, data.data)
    edited.data = data.data
    if grade:
        grader_vars[name] = edited
    publishers[name].publish(edited)
    if (log):
        log_file = log_files[name]
        log_file.write(str(clock_time.to_sec()) + ", internal: " + str(original) + ", edited: " + str(edited.data) + "\n")
        log_file.flush()
        if (verbose):
            log_print ("Logging %s data" % name)

def generate_cb(name):
    return (lambda data: cb_generic(name, data))

def generate_subscribers():
    global subscribers
    for name in tdef.sensor_names:
        sub_name = name + "_raw"
        cb = generate_cb(name)
        subscribers[name] = rospy.Subscriber(sub_name, tdef.sensor_types[name], cb)

    for name in tdef.actuator_names:
        sub_name = name + "_input"
        cb = generate_cb(name)
        subscribers[name] = rospy.Subscriber(sub_name, tdef.actuator_types[name], cb)

###Start of program
parser = argparse.ArgumentParser(description = "relay parser for Autonomous Systems")
parser.add_argument('-v', '--verbose', action = 'store_true')
parser.add_argument('-l', '--log', action = 'store_true')
parser.add_argument('-m', '--mode', default = "serial",
        choices = ['serial', 'sim', 'grade'],
        help = "if no mode given, serial is used")
parser.add_argument('--speedup', default = 1, type = float)
parser.add_argument('--baseline', default = "grading/baseline.txt")
parser.add_argument('--interference', default = None)
parser.add_argument('-t','--tracefile', default = None,
        help = "if --tracedir is also set, this argument is ignored")
parser.add_argument('-T','--tracedir', default = None)
parser.add_argument('--agent', default = 'none',
        help = "if agent is 'none', agent must be run externally")
parser.add_argument('--email', default = None,
        help = "email address to notify if restarting frequently")

args = parser.parse_args()

verbose = args.verbose
log = args.log
mode = args.mode
simulate = mode == "sim"
grade = mode == "grade"
ser = mode == "serial"
run_agent = (args.agent != "None") and (args.agent != "none")

num_restarts = 0
max_restarts = 5

### TODO: Implement sending emails (see https://realpython.com/python-send-email/)
def send_email():
    global args, num_restarts
    if (args.email != None):
        print("Sending email regarding restarts to " + args.email)

if grade and (args.tracefile == None and args.tracedir == None):
    print("no tracefile or tracedir given, run ./relay.py -h for usage")
    quit()

def terminate_gracefully():
    global core_p, serial_p, sim_p, agent_p
    if core_p != None:
        print("Terminating roscore")
        core_p.terminate()
        core_p.wait()
    if sim_p != None:
        print("Terminating sim")
        sim_p.terminate()
        sim_p.wait()
    if agent_p != None:
        print("Terminating agent")
        agent_p.terminate()
        agent_p.wait()
    if serial_p != None:
        print("Terminating serial")
        serial_p.terminate()
        serial_p.wait()
    quit()

#initialize trace file array
tracefiles = []
if args.tracedir != None:
    tracefiles = glob.glob(args.tracedir + "/*.trc")
else:
    tracefiles = [args.tracefile]

if log:
    gen_log_files()


interf.parse_interf(args.interference)

### Open logs for roscore and relay
core_log = open("Log/roscore.log", "a+", 0)
relay_log = open("Log/relay.log", "a+", 0)

### Start up roscore, redirecting output to logging files
core_p = sp.Popen("roscore", stdout = core_log, stderr = core_log)

### Begin relay node
if simulate:
    while not rosgraph.is_master_online():
        rospy.sleep(1) # Wait for roscore to start up
    rospy.set_param("use_sim_time", True)
rospy.init_node('relay', anonymous = True)

now = 0 if simulate else rospy.get_time()
last_ping = now

generate_publishers()
generate_subscribers()

### Health Ping callback function
### Records the most recent ping
def ping_cb(data):
    global last_ping, now
    last_ping = now

ping_sub = rospy.Subscriber('ping', Bool, ping_cb)

### Spawn subprocesses

### Simulator starts up Agent and Farduino
#TODO add baseline functionality
agent_p = None
if run_agent:
    agent_log = open("Log/agent.log", "a+", 0)
sim_p = None
serial_p = None

if simulate:
    sim_log = open("Log/simulator.log", "a+", 0)
    ### Initiates the Simulator and redirects output
    fard_args = ["--baseline", args.baseline, "--speedup", str(args.speedup)]
    if log:
        fard_args = fard_args + ["-l"]
    sim_p = sp.Popen(["python", "farduino.py"] + fard_args,
                     stdout = sim_log, stderr = sim_log)
    ### Initiates the Agent file and redirects output
    if run_agent:
        agent_p = sp.Popen(["python", args.agent],
                             stdout = agent_log, stderr = agent_log)
    print("waiting for nodes")
    rospy.sleep(2)
    print("ok")



### Running the real arduino proccess
elif mode == "serial":
    serial_log = open("Log/rosserial.log", "a+", 0)
    ### Initiates the Arduino and redirects output
    serial_p = sp.Popen(["rosrun", "rosserial_arduino",
        "serial_node.py", "/dev/ttyACM0"],
        stdout = serial_log, stderr = serial_log)
    ### Initiates the Agent file and redirects output
    if run_agent:
        agent_p = sp.Popen(["python", args.agent],
                             stdout = agent_log, stderr = agent_log)
    print("waiting for nodes")
    rospy.sleep(2)
    print("ok")



if (verbose):
    log_print("Spinning...")

### Loop for the entire system, should only ever break if grading
while len(tracefiles) > 0 or not grade:
    ### TODO add grading functionality
    ### We must begin health ping
    now= rospy.get_time()
    last_ping = now

    if grade:
        reload(grader)
        reload(interf)

        tfile = tracefiles.pop(0)
        dirname = os.path.dirname(tfile) + "/"

        grader.open_trace(tfile)
        if (grader.interf_file == ""):
            interf.parse_interf()
        else:
            interf.parse_interf(dirname + grader.interf_file)

        agent_p = sp.Popen(["python", args.agent],
            stdout = agent_log, stderr = agent_log)

        sim_log = open("Log/simulator.log", "a+", 0)
        ### Initiates the Simulator and redirects output

        exec(open(dirname + grader.bfile).read())
        grader_vars = init_actuators

        fard_args = ["--baseline", dirname + grader.bfile,
                     "--speedup", str(args.speedup)]
        if log:
            fard_args = fard_args + ["-l"]
        sim_p = sp.Popen(["python", "lib/farduino.py"] + fard_args,
                         stdout = sim_log, stderr = sim_log)
        print("running %s"%tfile)
        rospy.sleep(1)


    ### Loop for a single iteration of a grading scheme(infinite if not grading)
    while not rospy.core.is_shutdown():
        ### Check for input
        if sys.stdin in select.select([sys.stdin],[],[],0)[0]:
            input = sys.stdin.readline()
            if input[0] == 'q':
                terminate_gracefully()
            else:
                print("Usage: q (quit)")

        ### If not simulating get real time
        #print("spin")
        now = rospy.get_time()
        ### TODO add grading functionality
        if grade:
            grader.grader_vars = grader_vars
            cur_cmd = grader.run_command(rospy.Time.from_sec(now))
            if grader.finished:
                print("done")
                break


        if  run_agent and ((now - last_ping) > 360):
            log_print("no ping since %f, terminating..."%last_ping)
            last_ping = now
            agent_p.terminate()

        if (run_agent and (agent_p.poll() != None)):
            log_print("agent restarting...")
            agent_p = sp.Popen(["python", args.agent],
                               stdout = agent_log, stderr = agent_log)
            num_restarts += 1
            if (num_restarts == max_restarts): # Send just once
                send_email()

        rospy.sleep(tick_interval)
    if grade:
        sim_p.terminate()
        sim_p.wait()
        if run_agent:
            agent_p.terminate()
            agent_p.wait()
    else:
        break

core_p.terminate()

