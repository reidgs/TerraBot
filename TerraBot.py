#!/usr/bin/env python
import os, sys, glob, select, signal
import subprocess as sp
from std_msgs.msg import Int32,Bool,Float32,String,Int32MultiArray,Float32MultiArray, String
import argparse, time, getpass
from shutil import copyfile
import rospy, rosgraph
from lib import topic_def as tdef
from lib import interference as interf_mod
from lib import grader as grader_mod
from lib import send_email
from lib import sim_camera as cam
from lib.terrabot_utils import clock_time

### Default values for the optional variables
verbose = False
grade = False
log = False
simulate = False
still_running = True
run_agent = True
#tick_interval = 0.01
tick_interval = 0.5
#clock_time = rospy.Time(0)
interference = None
grader = None

#lists which will be populated based on the topic_def.py
log_files = {}
publishers = {}
subscribers = {}

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
    global now, grader, interference
    original = data.data
    edited = data
    edited.data = (original if not interference else
                   interf_mod.edit(name, original))
    #edited.data = data.data # Why is this here?

    if grade: grader.update(name, edited.data)

    publishers[name].publish(edited)
    if (log):
        log_file = log_files[name]
        log_file.write(clock_time(now) + ", internal: " +
                       str(original) + ", edited: " + str(edited.data) + "\n")
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
parser.add_argument('-g', '--graphics', default = False, action='store_true')
parser.add_argument('-s', '--speedup', default = 1, type = float)
parser.add_argument('-b', '--baseline', default = None)
parser.add_argument('-i', '--interference', default = None)
parser.add_argument('-t','--tracefile', default = None,
        help = "if --tracedir is also set, this argument is ignored")
parser.add_argument('-T','--tracedir', default = None)
parser.add_argument('-a', '--agent', default = 'none',
        help = "if agent is 'none', agent must be run externally")
parser.add_argument('-e', '--email', default = None,
        help = "email address to notify if restarting frequently")
parser.add_argument('-p', '--password', default = None,
        help = "email address password")

args = parser.parse_args()

verbose = args.verbose
log = args.log
mode = args.mode
simulate = mode == "sim"
grade = mode == "grade"
ser = mode == "serial"
run_agent = (args.agent != "None") and (args.agent != "none")
password = args.password

num_restarts = 0
max_restarts = 5

### Notify user when agent crashes or hangs multiple times
def notify_user():
    global args, password, num_restarts
    if (args.email != None):
        print("Sending email regarding restarts to " + args.email)
        send_email.send("autoterrabot@gmail.com", password, args.email,
                        "Problem with your TerraBot agent",
                        ("Your autonomous agent has had to be restarted %d times.  The current Terrabot time is %s.\r\n\r\nYou should probably contact the instructors to upload a new version.\r\n\r\nSincerely,\r\nTerraBot\r\n"
                         %(num_restarts, clock_time(rospy.get_time()))))

if (args.email != None and password == None):
    password = getpass.getpass("Password please for autoterrabot@gmail.com: ")

if grade and (args.tracefile == None and args.tracedir == None):
    print("no tracefile or tracedir given, run ./relay.py -h for usage")
    sys.exit()

if grade and not run_agent:
    print("grader must be ran with an agent")
    sys.exit()

def terminate (process, log_file):
    if (log_file != None): log_file.close()
    if (process.poll() == None):
        process.terminate()
        process.wait()

def terminate_core():
    global core_p, core_log
    if (core_p != None):
        print("Terminating roscore")
        terminate(core_p, core_log)
        core_p = None; core_log = None

def terminate_sim():
    global sim_p, sim_log
    if (sim_p != None):
        print("Terminating sim")
        terminate(sim_p, sim_log)
        sim_p = None; sim_log = None

def terminate_serial():
    global serial_p, serial_log
    if (serial_p != None):
        print("Terminating serial")
        terminate(serial_p, serial_log)
        serial_p = None; serial_log = None

def terminate_agent():
    global agent_p, agent_log
    if (agent_p != None):
        print("Terminating agent")
        terminate(agent_p, agent_log)
        agent_p = None; agent_log = None

def terminate_gracefully():
    terminate_agent()
    terminate_sim()
    terminate_serial()
    terminate_core()
    sys.exit()

#initialize trace file array
tracefiles = []
if args.tracedir != None:
    tracefiles = glob.glob(args.tracedir + "/*.trc")
else:
    tracefiles = [args.tracefile]

if log:
    gen_log_files()

### Open log file for roscore
core_log = open("Log/roscore.log", "a+", 0)

### Start up roscore, redirecting output to logging files
core_p = sp.Popen("roscore", stdout = core_log, stderr = core_log)

### Begin relay node
if simulate or grade: # Use simulated time if starting simulator
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
    global last_ping
    last_ping = rospy.get_time()
    print("  PING! %s" %clock_time(last_ping))

ping_sub = rospy.Subscriber('ping', Bool, ping_cb)

images = None
image_start_time = 1735000 # Should be in baseline
### Camera callback - take a picture and store in the given location
def camera_cb(data):
    global simulate, images
    
    if simulate:
        publishers['cam'].publish(data.data)
    elif grade:
        if (images == None): images = cam.get_images_from_directory("images")
        image = cam.find_image(rospy.get_time() + image_start_time, images)
        if (image == None):
            print("ERROR: No stored images found")
        else:
            print("Retrieving image %s, storing it in %s" %(image, data.data))
            copyfile(image, data.data)
    else:
        print("Taking an image, storing it in %s" %data.data)
#        sp.call("raspistill -n -md 2 -awb off -awbg 1,1 -ss 30000 -o %s"
        sp.call("raspistill -n -md 4 -awb auto -ss 30000 -rot 180 -o %s"
                % data.data, shell = True)

camera_sub = rospy.Subscriber('camera', String, camera_cb)

### Spawn subprocesses

sim_p = None
sim_log = None
agent_p = None
agent_log = None
serial_p = None
serial_log = None

### Initiates the Simulator and redirects output
def start_simulator():
    global sim_p, sim_log, args
    if (sim_log == None): sim_log = open("Log/simulator.log", "a+", 0)
    fard_args = ["--speedup", str(args.speedup)]
    if args.graphics: fard_args += ["--graphics"]
    if args.baseline: fard_args += ["--baseline", args.baseline]
    if log: fard_args = fard_args + ["-l"]
    sim_p = sp.Popen(["python", "lib/farduino.py"] + fard_args,
                     stdout = sim_log, 
                     stderr = sim_log)
    time.sleep(1) # chance to get started
    
def start_agent():
    global agent_p, agent_log, args
    if (agent_log == None): agent_log = open("Log/agent.log", "a+", 0)
    agent_p = sp.Popen(["python", args.agent], bufsize=0,
                       stdout = agent_log, stderr = agent_log)
    time.sleep(1) # chance to get started
    last_ping = rospy.get_time()

### Initiates the Arduino and redirects output
def start_serial():
    global serial_p, serial_log
    if (serial_log == None): serial_log = open("Log/rosserial.log", "a+", 0)
    serial_p = sp.Popen(["rosrun", "rosserial_arduino",
                         "serial_node.py", "/dev/ttyACM0"],
                        stdout = serial_log, stderr = serial_log)
    time.sleep(1) # chance to get started

### Simulator starts up Agent and Farduino
if simulate:
    print("waiting for nodes")
    start_simulator()
    ### Initiates the Agent file and redirects output
    if run_agent: start_agent()
    print("ok")

### Running the real arduino proccess
elif mode == "serial":
    print("waiting for nodes")
    start_serial()
    if run_agent: start_agent()
    print("ok")

if (verbose):
    log_print("Spinning...")

### Loop for the entire system, should only ever break if grading
while len(tracefiles) > 0 or not grade:
    ### TODO add grading functionality

    if grade:
        reload(grader_mod)
        reload(interf_mod)

        tfile = tracefiles.pop(0)
        dirname = os.path.dirname(tfile) + "/"

        grader = grader_mod.Grader(tfile)
        args.baseline = (None if not grader.baseline_file else
                         dirname + grader.baseline_file)
        args.interference = (None if not grader.interf_file else
                             dirname + grader.interf_file)
        start_simulator()
        start_agent()
        print("running %s"%tfile)

    ### We must begin health ping
    now= rospy.get_time()
    last_ping = now

    if (args.interference):
        interference = interf_mod.Interference(args.interference, now)

    ### Loop for a single iteration of a grading scheme(infinite if not grading)
    while not rospy.core.is_shutdown():
        ### Check for input
        if sys.stdin in select.select([sys.stdin],[],[],0)[0]:
            input = sys.stdin.readline()
            if input[0] == 'q':
                terminate_gracefully()
            else:
                print("Usage: q (quit)")

        now = rospy.get_time()
        if (interference): interference.update(now)
        if grade:
            cur_cmd = grader.run_command(now)
            if grader.finished():
                print("done")
                terminate_sim()
                terminate_agent()
                break

        # Ping at least once every 6 minutes, but need to adjust if speedup
        if  run_agent and ((now - last_ping) > max(360, 2*args.speedup)):
            log_print("no ping since %d (%d seconds), terminating..."
                      %(last_ping, now - last_ping))
            last_ping = now
            num_restarts += 1
            # For safety, make sure the pump is off
            publishers['wpump'].publish(False)
            if (num_restarts < max_restarts): # Send just once
                terminate(agent_p, None); agent_p = None
            else:
                notify_user()
                terminate_gracefully()

        if (run_agent and agent_p != None and agent_p.poll() != None):
            log_print("agent died")
            num_restarts += 1
            # For safety, make sure the pump is off
            publishers['wpump'].publish(False)
            if (num_restarts >= max_restarts): # Send just once
                notify_user()
                terminate_gracefully()

        if (run_agent and (agent_p == None or agent_p.poll() != None)):
            log_print("agent restarting...")
            start_agent()

        rospy.sleep(tick_interval)
    if grade:
        terminate_sim()
        if run_agent:
            terminate_agent()
    else:
        break

terminate_core()

