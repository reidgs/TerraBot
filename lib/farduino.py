#!/usr/bin/env python

#David Buffkin

### Import used files
import rclpy, rclpy.node
from std_msgs.msg import Int32,Bool,Float32,String,Int32MultiArray,Float32MultiArray
from topic_def import *
from rosgraph_msgs.msg import Clock
from rclpy.time import Time
import time
import argparse                                                    
from baseline import Baseline
from os.path import abspath

import environment as env
from render import Terrarium
from freqmsg import frommsg
from direct.stdpy import threading
import sys
from datetime import datetime
from terrabot_utils import clock_time, get_ros_time, set_use_sim_time

class Simulator(rclpy.node.Node):
    def __init__(self):
        super().__init__("Simulator")

### Parse arguments

parser = argparse.ArgumentParser(description='simulation parser for Autonomous Systems')
parser.add_argument('--baseline', type = str, default = None, nargs = "?")
parser.add_argument('--speedup', type = float, default = 1, nargs = "?")
parser.add_argument('--graphics', default = False, action = 'store_true')
parser.add_argument('-l', '--log', action = 'store_true')
args = parser.parse_args()

rclpy.init()

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
    simulator.create_subscription(Int32, 'speedup', speedup_cb, 1)
    for name in actuator_names: 
        if name != 'camera':    
            subscribers[name] = simulator.create_subscription(
                                             actuator_types[name],  
                name + '_raw', 
                                             actuator_cbs[name], 1)

## Sensor Setup

def generate_publishers():
    global publishers, clock_pub
    clock_pub = simulator.create_publisher(Clock, "/clock", 1)
    for name in sensor_names:
        publishers[name] = simulator.create_publisher(sensor_types[name],
                                                      name+'_raw', 100)
    
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
    float = Float32()
    float.data = env.params['volume'] / env.volume_rate
    publishers['level'].publish(float)
    
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
    # The sum of the two sensors is the weight; Make it slightly uneven
    weight = env.get_weight()/2
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

simulator = Simulator()
set_use_sim_time(simulator, True)

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
    clock = Clock()
    clock.clock.sec = now; clock.clock.nanosec = 0
    clock_pub.publish(clock) #Publish initial time
    while get_ros_time(simulator) == 0:
        rclpy.spin_once(simulator, timeout_sec=0.1)

    while doloop:
        
        rclpy.spin_once(simulator, timeout_sec=0.01)
        time.sleep(tick_time-0.01) # Wait for next tick
        
        speedup = min(default_speedup, #speedup should be maxed if pumping/fanning
                      (max_speedup_pump if env.params['wpump'] else default_speedup),
                      (max_speedup_fan if env.params['fan'] else default_speedup))

        now = get_ros_time(simulator)
        if (pump_last_on and not env.params['wpump']):
            pump_last_on = False
            pump_last_off_time = now
        elif (env.params['wpump']):
            pump_last_on = True
        if (now - pump_last_off_time < 10): speedup = 1
        clock = Clock()
        now += tick_time * speedup
        clock.clock.sec = int(now)
        clock.clock.nanosec = int((now - clock.clock.sec) * 1e9)
        clock_pub.publish(clock)

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

def camera_cb(data):
    global renderer
    renderer.takeAndStorePic(data.data)

#Steup camera subscriber
subscribers['camera'] = simulator.create_subscription( actuator_types['camera'],
                                                       'camera_raw', camera_cb, 1)

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
    
