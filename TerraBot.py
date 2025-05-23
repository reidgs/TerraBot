
#!/usr/bin/env python
import os, os.path as op, sys, select
import subprocess as sp
from std_msgs.msg import Int32,Bool,Float32,String,Int32MultiArray,Float32MultiArray, String
import argparse, time, getpass
from shutil import copyfile
import rospy, rosgraph
from lib import topic_def as tdef
from lib import interference as interf_mod
from lib import tester as tester_mod
from lib import send_email
from lib import sim_camera as cam
from lib.terrabot_utils import clock_time, time_since_midnight
from lib.baseline import Baseline
from math import exp
from os import makedirs

### Default values for the optional variables
verbose = False
grade = False
log = False
simulate = False
still_running = True
tick_interval = 0.5
interference = None
tester = None

#lists which will be populated based on the topic_def.py
log_files = {}
publishers = {}
subscribers = {}

terrabot_dir = op.dirname(op.abspath(__file__))
log_dir = op.join(terrabot_dir, "Log")
lib_dir = op.join(terrabot_dir, "lib")

### Update tester variables, if necessary
var_translations = {'smoist' : 'smoist',      'light'  : 'light',
                    'level' : 'wlevel',       'weight' : 'weight',
                    'temp'   : 'temperature', 'humid'  : 'humidity',
                    'led'    : 'led',         'wpump'  : 'wpump',
                    'fan'    : 'fan',         'camera' : 'camera',
                    'insolation' : 'insolation'}
def tester_update_var(var, value):
    global tester, var_translations
    if (tester):
        trans = var_translations[var]
        if (isinstance(value, tuple)):
            svalue = sum(value) if var == 'weight' else sum(value)/2
            tester.vars[trans+'_raw'] = value
            tester.vars[trans] = svalue
        else:
            tester.vars[trans] = value
        #print(var, value, tester.vars)

def tester_update_behaviors(behavior, enabled_p):
    global tester
    if (tester):
        if (enabled_p): tester.vars['enabled_behaviors'].add(behavior)
        else: tester.vars['enabled_behaviors'].discard(behavior)

def gen_log_files():
    global log_files

    prefix = time.strftime("%Y%m%d_%H%M%S") + ("_sim" if simulate else "")
    makedirs(op.join(log_dir, "Log_%s" %prefix))

    for name in tdef.sensor_names + tdef.actuator_names:
        file_name = op.join(log_dir, "Log_%s/%s_log.csv" % (prefix, name))
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

insolation = 0
last_light_reading = 0
light_level = 0

def cb_generic(name, data):
    global now, interference, light_level
    original = data.data
    edited = data
    edited.data = (original if not interference else
                   interference.edit(name, original))

    tester_update_var(name, original)

    if (name == 'light'): # Integrate light levels
        global last_light_reading, insolation
        now = rospy.get_time()
        if (last_light_reading > 0):
            light_level = (original[0] + original[1])/2.0
            dt = now - last_light_reading
            insolation += dt*light_level/3600.0
            #print("INSOLATION: %.2f %d %.2f" %(insolation, light_level, dt))
            tester_update_var('insolation', insolation)
        if (time_since_midnight(now) < time_since_midnight(last_light_reading)):
            #print("INSOLATION TODAY: %.1f" %insolation)
            insolation = 0 # Reset daily
        last_light_reading = now

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
parser = argparse.ArgumentParser(description = "TerraBot arg parser")
parser.add_argument('-v', '--verbose', action = 'store_true')
parser.add_argument('-l', '--log', action = 'store_true')
parser.add_argument('-m', '--mode', default = "serial",
        choices = ['serial', 'sim'],
        help = "if no mode given, serial is used")
parser.add_argument('-g', '--graphics', default = False, action='store_true')
parser.add_argument('-s', '--speedup', default = 1, type = float)
parser.add_argument('-b', '--baseline', default = None)
parser.add_argument('-i', '--interference', default = None)
parser.add_argument('-t','--test', default = None,
        help = "test execution using given tester file")
parser.add_argument('-e', '--email', default = None,
        help = "email address to notify if restarting frequently")
parser.add_argument('-p', '--password', default = None,
        help = "email address password")
parser.add_argument('-f', '--fixedshutter', default = None,
        help = "use fixed shutter speed")


args = parser.parse_args()

verbose = args.verbose
log = args.log
mode = args.mode
tester_file = args.test
simulate = mode == "sim"
password = args.password
fixed_shutter = (args.fixedshutter if args.fixedshutter == None else
                 int(args.fixedshutter))

num_restarts = 0
max_restarts = 5

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
        print("Turning off actuators")
        publishers['led'].publish(0)
        publishers['wpump'].publish(0)
        publishers['fan'].publish(0)

        print("Terminating serial")
        terminate(serial_p, serial_log)
        serial_p = None; serial_log = None


def terminate_gracefully():
    terminate_sim()
    terminate_serial()
    terminate_core()
    sys.exit()

if log:
    gen_log_files()

### Open log file for roscore
core_log = open(op.join(log_dir, "roscore.log"), "a+")

### Start up roscore, redirecting output to logging files
core_p = sp.Popen("roscore", stdout = core_log, stderr = core_log)

### Begin relay node
if simulate: # Use simulated time if starting simulator
    while not rosgraph.is_master_online():
        rospy.sleep(1) # Wait for roscore to start up
    rospy.set_param("use_sim_time", True)
rospy.init_node('TerraBot', anonymous = True)

generate_publishers()
generate_subscribers()

images = None
image_start_time = 1735000 # Should be in baseline
### Camera callback - take a picture and store in the given location
def camera_cb(data):
    global simulate, images, fixed_shutter
    
    print("Taking an image at %s, storing it in %s"
          %(clock_time(rospy.get_time()), data.data))
    if simulate:
        publishers['camera'].publish(data.data)
    else:
        # Shutter speed in microseconds, 2.8 aperture
        shutter_speed = ((fixed_shutter if fixed_shutter != None else
                          int((1e6*2.8*2.8)/(exp(3.32)*(max(1,light_level)**0.655)))))
        print("LIGHT:", light_level, "SHUTTER: ", shutter_speed)
#        sp.call("raspistill -n -md 2 -awb off -awbg 1,1 -ss 30000 -o %s"
#        sp.call("raspistill -n -md 4 -awb auto -ss 30000 -rot 180 -o %s"
        
        sp.call("raspistill -n -md 4 -awb auto -ss %d -o %s"
                %(shutter_speed, data.data), shell = True)
    tester_update_var('camera', data.data)

camera_sub = rospy.Subscriber('camera', String, camera_cb)

def enable_cb (data):
    #print("Enabling behavior:", data.data)
    tester_update_behaviors(data.data, True)

def disable_cb (data):
    #print("Disabling behavior:", data.data)
    tester_update_behaviors(data.data, False)

enable_sub = rospy.Subscriber('enable', String, enable_cb)
disable_sub = rospy.Subscriber('disable', String, disable_cb)

### Spawn subprocesses

sim_p = None
sim_log = None
serial_p = None
serial_log = None

### Initiates the Simulator and redirects output
def start_simulator():
    global sim_p, sim_log, args
    if (sim_log == None): sim_log = open(op.join(log_dir, "simulator.log"), "a+")
    fard_args = ["--speedup", str(args.speedup)]
    if args.graphics: fard_args += ["--graphics"]
    if args.baseline: 
        try: # In case of syntax errors, try parsing here first
            Baseline(op.abspath(args.baseline))
        except Exception as inst:
            print("start_simulator: %s" %str(inst.args))
            terminate_gracefully()
        fard_args += ["--baseline", args.baseline]
    if log: fard_args = fard_args + ["-l"]
    sim_p = sp.Popen(["python", op.join(lib_dir, "farduino.py")] + fard_args,
                     stdout = sim_log, stderr = sim_log)
    time.sleep(1) # chance to get started

### Initiates the Arduino and redirects output
def start_serial():
    global serial_p, serial_log
    if (serial_log == None): serial_log = open(op.join(log_dir, "rosserial.log"), "a+")
    serial_p = sp.Popen(["rosrun", "rosserial_arduino",
                         "serial_node.py", "/dev/ttyACM0"],
                        stdout = serial_log, stderr = serial_log)
    time.sleep(1) # chance to get started
    print("started serial")

def adjust_path(pathname, dirname):
    return (pathname if op.isabs(pathname) else
            op.abspath(dirname + pathname))
    
if tester_file:
    tester = tester_mod.Tester()
    try:
        tester.parse_file(tester_file)
    except Exception as inst:
        print("Tester parse: %s" %str(inst.args))
        terminate_gracefully()
    #tester.display()
    dirname = op.dirname(tester_file) + "/"
    # Command line option overrides test file
    if (not args.baseline and tester.baseline_file):
        args.baseline = adjust_path(tester.baseline_file, dirname)
        print('baseline', args.baseline)
    # Command line option overrides test file
    if (not args.interference and tester.interf_file):
        args.interference = adjust_path(tester.interf_file, dirname)

### Start up arduino/simulator
print("Waiting for nodes")
if simulate:
    print("  Starting simulator")
    start_simulator()
else:
    print("  Starting hardware")
    start_serial()
# Wait for clock to start up correctly
while rospy.get_time() == 0: rospy.sleep(0.1)
now = rospy.get_time()

print("System started")

if (tester != None):
    tester.init_constraints(now)

if (args.interference):
    interference = interf_mod.Interference(args.interference, now)

### Main loop
while not rospy.core.is_shutdown():
    ### Check for input
    if sys.stdin in select.select([sys.stdin],[],[],0)[0]:
        input = sys.stdin.readline()
        if input[0] == 'q':
            terminate_gracefully()
        if input[0] == 't':
            print("Current time: %s" %clock_time(now))
        else:
            print("Usage: q (quit); t (current time)")

    now = rospy.get_time()
    if (interference): interference.update(now)
    if tester:
        tester.vars['time'] = now
        tester.vars['mtime'] = time_since_midnight(now)
        tester.process_constraints(now)
        tester_update_var('camera', None) # Camera should not be latched
        if tester.finished(now):
            print("Done testing!")
            if (tester.end_status() == 'QUIT'): terminate_gracefully()
            else: tester = None

    rospy.sleep(tick_interval)
    # End while loop


