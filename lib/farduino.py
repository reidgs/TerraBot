#!/usr/bin/env python

### Unplaced TODO list:
#implement max speedup
#everything Baseline: {plant options: HEALTHY, DEAD, SOSO; time}: 
#Add logging support
#Consider other command line args




### Import used files
#TODO: figure out which of these I actually need, these are copied from the other one
import rospy
from std_msgs.msg import Int32,Bool,Float32,String,Int32MultiArray,Float32MultiArray
from topic_def import *
from rosgraph_msgs.msg import Clock
import time
from datetime import datetime
import argparse                                                    
from baseline import Baseline
from os.path import abspath
from terrabot_utils import clock_time, time_since_midnight

import environment as env
from render import Terrarium
from freqmsg import frommsg
import threading
import sys


### Parse arguments

parser = argparse.ArgumentParser(description='simulation parser for Autonomous Systems')
parser.add_argument('--baseline', type = str, default = None, nargs = "?") #Use type=file instead?
parser.add_argument('--speedup', type = float, default = 1, nargs = "?")
parser.add_argument('--graphics', default = False, action = 'store_true')
parser.add_argument('-l', '--log', action = 'store_true')
args = parser.parse_args()



### Important Variables

default_speedup = args.speedup
publishers = {}
subscribers = {}
clock_pub = None



### Handle Sub/Pub

## Actuator Callbacks

def freq_cb(data):
    #Parse and update sensor frequency
    name, freq = frommsg(data.data)
    sensor_timing[name][1] = freq
    
    
def speedup_cb(data):
    global speedup, default_speedup
    default_speedup = data.data
    speedup = default_speedup
    
actuator_cbs = { 'led' : (lambda data : env.params.update({'led' : data.data})),
                 'wpump' : (lambda data : env.params.update({'wpump' : data.data})),
                 'fan' : (lambda data : env.params.update({'fan' : data.data})),
                 'freq' : freq_cb}
                 
## Actuator Setup

def generate_subscribers():
    global subscribers
    rospy.Subscriber('speedup', Int32, speedup_cb)
    for name in actuator_names: 
        if name != 'cam':    
            subscribers[name] = rospy.Subscriber(name + '_raw', 
                                             actuator_types[name],  
                                             actuator_cbs[name])

## Sensor Setup

def generate_publishers():
    global publishers, clock_pub
    clock_pub = rospy.Publisher("clock", Clock, latch = True, queue_size = 1)
    for name in sensor_names:
        publishers[name] = rospy.Publisher(name + '_raw', sensor_types[name],
                                           latch = True, queue_size = 100)
    


### GRAPHICS ###

#create the app(ShowBase)

#task to rerender every frame, limit framerate to 1/tick_time or whatever

#Things to take into account when rendering:  
# light, tankwater, soil should be darker the more moist,
# Led should show as on/off, same for wpump, fan
# volume in reservoir
#Plants will be necessary later



### SENSORS ###

# The first element is the current time till resensing, and the second is the chosen frequency
#TODO: Use better default values
sensor_timing = { 'smoist' : [0.0, 1.0],
                  'cur' : [0.0, 1.0],
                  'light' : [0.0, 1.0],
                  'level' : [0.0, 1.0],
                  'temp' : [0.0, 1.0],
                  'humid' : [0.0, 1.0]}
    
## Sensor functions
#These functions sense from the environment and publish to their topic

def sense_smoist():
    #should be ~2 * soil water content?
    s_array = Int32MultiArray()
    s_array.data = [int(env.params['soilwater'] * 2), \
                    int(env.params['soilwater'] * 2)]
    publishers['smoist'].publish(s_array)
    
def sense_cur():
    c_array = Float32MultiArray()
    c_array.data = [env.get_cur(), env.params['energy']]
    publishers['cur'].publish(c_array)

def sense_light():
    l_array = Int32MultiArray()
    l_array.data = [int(env.light_level(env.tank_width / 2)), \
                    int(env.light_level(env.tank_width / 2))]
    publishers['light'].publish(l_array)
    
def sense_level():
    publishers['level'].publish(env.params['volume'] / env.volume_rate)
    
def sense_temp():
    t_array = Int32MultiArray()
    t_array.data = [int(env.params['temperature']), \
                    int(env.params['temperature'])]
    publishers['temp'].publish(t_array)

def sense_humid():
    #Should be ~? * air water content ??
    h_array = Int32MultiArray()
    h_array.data = [int(env.params['airwater'] ), \
                    int(env.params['airwater'] )]
    publishers['humid'].publish(h_array)
              
def sensor_forward_time(duration):
    for name in sensor_names:
        sensor_timing[name][0] += duration #Update cooldown
        if sensor_timing[name][0] >= sensor_timing[name][1] : #Check to see if cooldown is over
            sensor_timing[name][0] = 0 #Reset cooldown
            eval("sense_%s()" % name) #Call the sensor function



### RUNTIME ###

## handle env init stuff

# extract baseline values
bl = None
if args.baseline:
    try:
        bl = Baseline(abspath(args.baseline))
    except:
        print('baseline file %s not found or parse error' %args.baseline)
        exit()

env.init(bl)

## handle ROS init stuff

rospy.set_param("use_sim_time", True)
rospy.init_node('Simulator', anonymous=True)

generate_publishers()
generate_subscribers()


### Sim loop (Put here for threading purposes)
doloop = True

def sim_loop():   
    global doloop   
    tick_time = .2  # This is how long between runs of the below loop in seconds
    speedup = default_speedup                
    now = 0.0 #TODO change to baseline initial time? (Also: this is in seconds)
    clock_pub.publish(rospy.Time.from_sec(now)) #Publish initial time (I think this is unnecessary)
    max_speedup_pump = 1
    max_speedup_fan = 5

    time.sleep(1) #give a sec
     
    ## Loop TODO should wait for an agent before running?

    while not rospy.core.is_shutdown() and doloop:
        
        time.sleep(tick_time) # Wait for next tick
        
        speedup = min(default_speedup, #speedup should be maxed if pumping/fanning
                      (max_speedup_pump if env.params['wpump'] else default_speedup),
                      (max_speedup_fan if env.params['fan'] else default_speedup))
       
        now = rospy.get_time()
        clock_pub.publish(rospy.Time.from_sec(now + (tick_time * speedup)))
        
        #DO STUFF 
        #move sensor time forward (speed times would be better but maybe too computational)
        sensor_forward_time(tick_time * speedup)
        #move env forward (all at once, or speed times, tick interval each?)
        env.forward_time(tick_time * speedup)
        #rerender to the viewing window. Or, just have it done automatically by panda, and set framerate to 1 / tick_time.
        if args.graphics:
            renderer.update_env_params(env.params, speedup)
    #Stop panda window?
    if args.graphics:
        renderer.userExit()
    
if not args.graphics:
    sim_loop()
else:


    ##This will need to change to accomodate no graphics
    #Init graphics
       
    renderer = Terrarium()
    renderer.update_env_params(env.params, default_speedup)

    def cam_cb(data):
        global renderer
        #print(data.data)
        renderer.takeAndStorePic(data.data)

    #Steup cam subscriber
    subscribers['cam'] = rospy.Subscriber('cam_raw', 
                                                 actuator_types['cam'],  
                                                 cam_cb)


    #Start sim loop THEN panda

    import signal
    def handler(signum, frame):
        global thread, doloop
        #print("HERE")
        doloop = False
        thread.join()
        sys.exit()
        
    signal.signal(signal.SIGTERM, handler)

    thread = threading.Thread(target=sim_loop)
    thread.start()

    if True:
        renderer.run()
    
    
