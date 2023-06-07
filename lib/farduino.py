#!/usr/bin/env python

#David Buffkin

### Import used files
import rospy
from std_msgs.msg import Int32,Bool,Float32,String,Int32MultiArray,Float32MultiArray
from topic_def import *
from rosgraph_msgs.msg import Clock
import time
import argparse                                                    
from baseline import Baseline
from os.path import abspath

import environment as env
from render import Terrarium
from freqmsg import frommsg
import threading
import sys
from datetime import datetime
from terrabot_utils import clock_time

### Parse arguments

parser = argparse.ArgumentParser(description='simulation parser for Autonomous Systems')
parser.add_argument('--baseline', type = str, default = None, nargs = "?")
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
    if freq == 0:
        sensor_timing[name][1] = 99999999
    else:
        sensor_timing[name][1] = 1 / freq
    
    
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
    

### SENSORS ###

# The first element is the current time till resensing, and the second is the chosen frequency
sensor_timing = { 'smoist' : [0.0, 1.0],
                  'cur' : [0.0, 1.0],
                  'light' : [0.0, 1.0],
                  'level' : [0.0, 1.0],
                  'temp' : [0.0, 1.0],
                  'humid' : [0.0, 1.0],
                  'weight' : [0.0, 1.0]}
    
## Sensor functions
#These functions sense from the environment and publish to their topic

def sense_smoist():
    #should be ~2 * soil water content?
    s_array = Int32MultiArray()
    s_array.data = [int(env.params['soilwater'] * 2)] * 2
    publishers['smoist'].publish(s_array)
    
def sense_cur():
    c_array = Float32MultiArray()
    c_array.data = [env.get_cur(), env.params['energy']]
    publishers['cur'].publish(c_array)

def sense_light():
    l_array = Int32MultiArray()
    l_array.data = [int(env.light_level(env.tank_width / 2))] * 2
    publishers['light'].publish(l_array)
    
def sense_level():
    publishers['level'].publish(env.params['volume'] / env.volume_rate)
    
def sense_temp():
    t_array = Int32MultiArray()
    t_array.data = [int(env.params['temperature'])] * 2
    publishers['temp'].publish(t_array)

def sense_humid():
    h_array = Int32MultiArray()
    h_array.data = [round(env.params['humidity'])] * 2
    publishers['humid'].publish(h_array)

def sense_weight():
    w_array = Float32MultiArray()
    # The average of the two sensors is the weight; Make it slightly uneven
    weight = env.get_weight()
    w_array.data = [weight*0.9, weight*1.1]
    publishers['weight'].publish(w_array)
              
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

max_speedup_pump = 1
max_speedup_fan = 100

## handle ROS init stuff

rospy.set_param("use_sim_time", True)
rospy.init_node('Simulator', anonymous=True)

generate_publishers()
generate_subscribers()


### Sim loop (Put here for threading purposes)
doloop = True

t0 = int(datetime(2000, 1, 1).strftime('%s')) # Initialize to 01-00:00:00, for display purposes

def sim_loop():   
    global doloop
    tick_time = .25  # This is how long between runs of the below loop in seconds
    speedup = default_speedup                
    pump_last_on = False
    pump_last_off_time = 0
    now = t0
    if bl is not None:
        now += bl.params['start']
    clock_pub.publish(rospy.Time.from_sec(now)) #Publish initial time

    time.sleep(1) #give a sec
     

    while not rospy.core.is_shutdown() and doloop:
        
        time.sleep(tick_time) # Wait for next tick
        
        speedup = min(default_speedup, #speedup should be maxed if pumping/fanning
                      (max_speedup_pump if env.params['wpump'] else default_speedup),
                      (max_speedup_fan if env.params['fan'] else default_speedup))

        now = rospy.get_time()
        if (pump_last_on and not env.params['wpump']):
            pump_last_on = False
            pump_last_off_time = now
        elif (env.params['wpump']):
            pump_last_on = True
        if (now - pump_last_off_time < 10): speedup = 1
        clock_pub.publish(rospy.Time.from_sec(now + (tick_time * speedup)))
        
        #DO STUFF 
        #move env forward
        duration = env.forward_time(tick_time * speedup)
        #move sensor time forward
        sensor_forward_time(duration)
        
        #rerender to the viewing window
        renderer.update_env_params(env.params, speedup, env.light_average(),
                                   env.get_weight())
    #Stop panda window
    if args.graphics:
        renderer.userExit()
    
    
#Init graphics
    
droop = 0
lankiness = 0
plant_health = 1
age = 0
if bl is not None:
    age = bl.params['start'] 
    droop = bl.params['leaf_droop']
    lankiness = bl.params['lankiness']
    plant_health = bl.params['plant_health']

renderer = Terrarium(args.graphics, t0, age, droop, lankiness, plant_health)
env.params['time'] = age
renderer.update_env_params(env.params, default_speedup, env.light_average(),
                           env.get_weight())

def cam_cb(data):
    global renderer
    renderer.takeAndStorePic(data.data)

#Steup cam subscriber
subscribers['cam'] = rospy.Subscriber('cam_raw', actuator_types['cam'],  
                                      cam_cb)

#Start sim loop THEN panda

import signal
def handler(signum, frame):
    global thread, doloop
    doloop = False
    thread.join()
    sys.exit()
    
signal.signal(signal.SIGTERM, handler)

thread = threading.Thread(target=sim_loop)
thread.start()

renderer.run()
    
# :)
    
