#!/usr/bin/env python

#David Buffkin
# Updated to ROS2 by Reid Simmons, May/June 2025

### Import used files
import rclpy, rclpy.node
from std_msgs.msg import Int32
from rosgraph_msgs.msg import Clock
import time, sys, argparse, os.path, signal
from functools import partial
from threading import Thread, Lock
from datetime import datetime

from topic_def import *
from freqmsg import frommsg
from terrabot_utils import clock_time, get_ros_time, set_use_sim_time
import environment as env
from render import Terrarium
from baseline import Baseline

class Simulator(rclpy.node.Node):
    def __init__(self, speedup):
        super().__init__("Simulator")
        self.default_speedup = self.speedup = speedup
        set_use_sim_time(self, True)
        self.clock_pub = None
        self.lock = Lock()
        self.received = {}
        self.generate_publishers()
        self.generate_subscribers()

    ### Handle Sub/Pub

    ## Actuator Callbacks
    def action_cb(self, action, data):
        # To avoid collisions, just squirrel the data away;
        # Data is processed by update_actions
        with self.lock: self.received[action] = data

    def generate_subscribers(self):
        for name in actuator_names: 
            self.create_subscription(actuator_types[name], name+'_raw',
                                     partial(self.action_cb, name), 1)
        self.create_subscription(Int32, 'speedup', 
                                 partial(self.action_cb, 'speedup'), 1)
        
    def update_actions(self):
        global renderer
        with self.lock:
            for action in self.received:
                data = self.received[action]
                if action == 'freq':
                    # Parse and update sensor frequency
                    name, freq = frommsg(data.data)
                    sensor_timing[name][1] = (99999999 if freq == 0 else 1/freq)
                elif action == 'speedup':
                    self.default_speedup = self.speedup = data.data
                elif action == 'camera':
                    renderer.takeAndStorePic(data.data)
                else:
                    env.params.update({action : data.data})
            self.received = {}

    ## Sensor Setup

    def generate_publishers(self):
        self.clock_pub = self.create_publisher(Clock, "/clock", 1)
        self.pubs = {}
        for name in sensor_names:
            self.pubs[name] = self.create_publisher(sensor_types[name],
                                                    name+'_raw', 100)
    
    ### Handling Sensors ###

    # The first element is the current time till resensing,
    #   and the second is the chosen frequency
    sensor_timing = { 'smoist' : [0.0, 1.0],
                      'cur' : [0.0, 1.0],
                      'light' : [0.0, 1.0],
                      'level' : [0.0, 1.0],
                      'temp' : [0.0, 1.0],
                      'humid' : [0.0, 1.0],
                      'weight' : [0.0, 1.0]}
    
    def publish (self, name, value):
        with self.lock:
            msg = sensor_types[name](data=value)
            self.pubs[name].publish(msg)

    # These functions sense from the environment and publish to their topic

    def sense_smoist(self):
        # should be ~2 * soil water content?
        self.publish('smoist', [int(env.params['soilwater'] * 2)] * 2)

    def sense_light(self):
        self.publish('light', [int(env.light_level(env.tank_width / 2))] * 2)
    
    def sense_level(self):
        self.publish('level', env.params['volume'] / env.volume_rate)
    
    def sense_temp(self):
        self.publish('temp', [int(env.params['temperature'])] * 2)

    def sense_humid(self):
        self.publish('humid', [round(env.params['humidity'])] * 2)

    def sense_weight(self):
        # The sum of the two sensors is the weight; Make it slightly uneven
        weight = env.get_weight()/2
        self.publish('weight', [weight*0.9, weight*1.1])
              
    def sensor_forward_time(self, duration):
        for name in sensor_names:
            self.sensor_timing[name][0] += duration #Update cooldown
            # Check to see if cooldown is over
            if self.sensor_timing[name][0] >= self.sensor_timing[name][1] :
                self.sensor_timing[name][0] = 0 #Reset cooldown
                eval("self.sense_%s()" % name) #Call the sensor function

def publish_clock(sim, now):
    clock = Clock()
    clock.clock.sec = int(now)
    clock.clock.nanosec = int((now - clock.clock.sec) * 1e9)
    sim.clock_pub.publish(clock)

# Have to use this, rather than spin(), which seems to get blocked by panda3d
def spin_fn(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    print("Spin thread exiting")

def sim_loop():
    global doloop, simulator, renderer
    tick_time = .25  # This is how long between runs of the below loop in seconds
    simulator.speedup = simulator.default_speedup
    pump_last_on = False
    pump_last_off_time = 0
    now = t0
    if bl is not None:
        now += bl.params['start']
    publish_clock(simulator, now) # Publish initial time

    while doloop:

        time.sleep(tick_time) # Wait for next tick

        simulator.update_actions()
        
        # speedup should be maxed if pumping/fanning
        simulator.speedup = min(simulator.default_speedup,
                                (max_speedup_pump if env.params['wpump'] else
                                 simulator.default_speedup),
                                (max_speedup_fan if env.params['fan'] else
                                 simulator.default_speedup))

        #now = get_ros_time(simulator)
        if (pump_last_on and not env.params['wpump']):
            pump_last_on = False
            pump_last_off_time = now
        elif (env.params['wpump']):
            pump_last_on = True
        if (now - pump_last_off_time < 10):
            simulator.speedup = 1
        now += tick_time * simulator.speedup
        publish_clock(simulator, now)

        #DO STUFF 
        #move env forward
        duration = env.forward_time(tick_time * simulator.speedup)
        #move sensor time forward
        simulator.sensor_forward_time(duration)
        
        #rerender to the viewing window
        renderer.update_env_params(env.params, simulator.speedup,
                                   env.light_average(), env.get_weight())
    #Stop panda window
    if renderer is not None:
        renderer.userExit()

#Init graphics
def init_graphics(sim, use_graphics):
    droop = 0
    lankiness = 0
    plant_health = 1
    age = 0
    if bl is not None:
        age = bl.params['start'] 
        droop = bl.params['leaf_droop']
        lankiness = bl.params['lankiness']
        plant_health = bl.params['plant_health']

    renderer = Terrarium(use_graphics, t0, age, droop, lankiness, plant_health)
    env.params['time'] = age
    renderer.update_env_params(env.params, sim.default_speedup,
                               env.light_average(), env.get_weight())
    return renderer

def handler(signum, frame):
    global thread, doloop, spin_thread
    doloop = False
    thread.join()
    rclpy.shutdown()
    spin_thread.join()
    sys.exit()

### Parse arguments
parser = argparse.ArgumentParser(description='simulation parser for Autonomous Systems')
parser.add_argument('--baseline', type = str, default = None, nargs = "?")
parser.add_argument('--speedup', type = float, default = 1, nargs = "?")
parser.add_argument('--graphics', default = False, action = 'store_true')
parser.add_argument('-l', '--log', action = 'store_true')
args = parser.parse_args()

### RUNTIME ###

# extract baseline values
bl = None
if args.baseline:
    try:
        bl = Baseline(os.path.abspath(args.baseline))
    except:
        print('baseline file %s not found or parse error' %args.baseline)
        exit()

## handle env init stuff
env.init(bl)

max_speedup_pump = 1
max_speedup_fan = 100

## handle ROS init stuff
rclpy.init()
simulator = Simulator(args.speedup)
# Put the message handler in its own thread
spin_thread = Thread(target=spin_fn, args=(simulator,), daemon=True)
spin_thread.start()

### Sim loop (Put here for threading purposes)
doloop = True

t0 = int(datetime(2000, 1, 1).strftime('%s')) # Initialize to 01-00:00:00, for display purposes

#Start sim loop THEN panda
signal.signal(signal.SIGTERM, handler)

renderer = init_graphics(simulator, args.graphics)
thread = Thread(target=sim_loop)
thread.start()

renderer.run()
    
# :)
    
