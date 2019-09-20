#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32,Bool,Float32,String,Int32MultiArray,Float32MultiArray
from topic_def import *
from rosgraph_msgs.msg import Clock
import time
from datetime import datetime
from math import sqrt
import argparse
import baseline
from terrabot_utils import clock_time, time_since_midnight

parser = argparse.ArgumentParser(description = "simulator parser for Autonomous Systems")
parser.add_argument('--baseline', type = str, default = None, nargs = "?")
parser.add_argument('--speedup', type = float, default = 1, nargs = "?")
parser.add_argument('-l', '--log', action = 'store_true')
args = parser.parse_args()
is_logging = args.log

### clock_start=0 is defined to be midnight on the first day
clock_start = 0
actuator_vars = { 'led'   : 0,
                  'wpump' : False,
                  'fan'   : False,
                  'freq'  : 10.0
                  }
internal_vars = { 'light'       : [10,100],
                  'temperature' : [20,20],
                  'humidity'    : [40,50],
                  'current'     : [0.0,0.0],
                  'smoist'      : [350,350],
#                  'volume'      : 1000.0 # ml (small reservoir)
                  'volume'      : 3000.0 # ml (large reservoir)
                  }
publishers = {}
subscribers = {}
clock_pub = None

###CONSTANTS
evap_rate = 10/3600.0 # moisture sensor decrease 10 units/hour
fan_evap_rate = 3*evap_rate # moisture decrease much faster when fans are on
soak_rate = 2 # moisture sensor increases 2 units/ml of water
flow_rate = 3.5 # ml/sec
#volume_rate = 6.25 # ml/mm in the (small) reservoir
volume_rate = 18 # ml/mm in the (large) reservoir
humidity_rate = 10/3600.0 # humidity sensor increases 20 units/hour
dehumidity_rate = 20/60.0 # fan decreases humidity by 20 units/minute
warming_rate = 1/3600.0 # temperature increases 1 degree Celcius/hour with lights on
cooling_rate = 1/60.0 # fan decreases temperature by 1 unit/minute
led_rate = 600/255.0 # how much each led level increases the light level
led_current = 3.2/255
pump_current = .2
fan_current = .06

max_speedup_pump = 1
max_speedup_fans = 5

def generate_publishers():
    global publishers, clock_pub
    clock_pub = rospy.Publisher("clock", Clock, latch = True, queue_size = 1)
    for name in sensor_names:
        pub_name = name + "_raw"
        publishers[name] = rospy.Publisher(
                            pub_name, sensor_types[name],
                            latch = True, queue_size = 100)

#used when generating subs
def update_keyval(dictionary, name, data):
    dictionary[name] = data.data

def generate_cb(dictionary, name):
    return (lambda data: update_keyval(dictionary,name,data))

def generate_subscribers():
    global subscribers
    rospy.Subscriber('speedup', Int32, speedup_cb)    
    for name in actuator_names:
        sub_name = name + "_raw"
        subscribers[name] = rospy.Subscriber(sub_name,
                                actuator_types[name],
                                generate_cb(actuator_vars, name))

default_speedup = args.speedup
speedup = default_speedup

### Change the speedup interactively
def speedup_cb(data):
    global default_speedup
    speedup = data.data
    if (default_speedup != speedup):
        print("speedup_pub %d" %speedup)
        default_speedup = speedup

### When days start and end (for simplicity, start and end on the hour)
morning_hour = 7  # 7am
evening_hour = 19 # 7pm

# Assumes that daylight hours do not span midnight
def is_daylight(time):
    global morning_hour, evening_hour
    dtime = datetime.fromtimestamp(time)
    return morning_hour <= dtime.hour and dtime.hour < evening_hour

light_level_day = 550 # Max ambient light in daytime
light_level_night = 5 # Ambient light in nighttime

last_print = 0
### Light increases from morning to noon and then decreases (non-linearally)
def amb_light(time):
    global morning_hour, evening_hour, light_level_day, light_level_night
    dl = light_level_night
    if (is_daylight(time)):
        dt = time_since_midnight(time)
        dl += ((light_level_day - light_level_night) *
               sqrt(1 - abs(1 - ((dt - 3600*morning_hour)/
                                 (3600*(evening_hour - morning_hour)/2)))))
    if is_logging:
        global last_print, clock_start
        if (time - last_print > 1800):
            last_print = time
            print(" Ambient: %.1f (%s)" %(dl, clock_time(time)))
    return dl

if is_logging: print(" Clock start: %s" %clock_time(clock_start))

### INTERNAL UPDATE FUNCTIONS ###
def light_update(cur_interval):
    for i in range(2):
        val = led_rate*actuator_vars['led'] + amb_light(rospy.get_time())
        internal_vars['light'][i] = min(600, max(0, val))

def volume_update(cur_interval):
    if actuator_vars['wpump']:
        internal_vars['volume'] -= cur_interval * flow_rate
        internal_vars['volume'] = max(0, internal_vars['volume'])

def humidity_update(cur_interval):
    for i in range(2):
        val = internal_vars['humidity'][i]
        if actuator_vars['fan']:
            val -= cur_interval * dehumidity_rate
        # The moister it is, the faster humidity increases
        val += cur_interval * humidity_rate * internal_vars['smoist'][i]/600
        internal_vars['humidity'][i] = min(100, max(0, val))

def temperature_update(cur_interval):
    for i in range(2):
        val = internal_vars['temperature'][i]
        if actuator_vars['fan']:
            val -= cur_interval * cooling_rate
        val += (cur_interval * warming_rate * actuator_vars['led']/255.0)
        internal_vars['temperature'][i] = min(40, max(15, val))

def current_update(cur_interval):
    val = (5.0 + led_current * actuator_vars['led'] +
           (pump_current if actuator_vars['wpump'] else 0) +
           (fan_current if actuator_vars['fan'] else 0))
    internal_vars['current'][0] = val
    # energy += power; power = current* voltage
    internal_vars['current'][1] += cur_interval * (val*12)/1000.0

def smoist_update(cur_interval):
    for i in range(2):
        val = internal_vars['smoist'][i]
        if (actuator_vars['wpump'] and internal_vars['volume'] > 0):
            val += cur_interval * soak_rate * flow_rate
        val -= cur_interval * (fan_evap_rate if actuator_vars['fan']
                               else evap_rate)
        internal_vars['smoist'][i] = min(1000, max(0, val))


update_funcs = {
    'volume'   : volume_update,
    'light' : light_update,
    'temperature' : temperature_update,
    'humidity'  : humidity_update,
    'current' : current_update,
    'smoist'      : smoist_update
}

### INTERNAL TO SENSOR TRANSLATION ###

def get_cur():
    c_array = Float32MultiArray()
    c_array.data = [float(internal_vars['current'][0]), \
                    float(internal_vars['current'][1])]
    return c_array

def get_light():
    l_array = Int32MultiArray()
    l_array.data = [int(internal_vars['light'][0]), \
                    int(internal_vars['light'][1])] 
    return l_array

def get_level():
    # Level in mms
    return float(internal_vars['volume'] / volume_rate)

def get_temp():
    t_array = Int32MultiArray()
    t_array.data = [int(internal_vars['temperature'][0]), \
                    int(internal_vars['temperature'][1])]
    return t_array

def get_humid():
    h_array = Int32MultiArray()
    h_array.data = [int(internal_vars['humidity'][0]), \
                    int(internal_vars['humidity'][1])]
    return h_array

def get_smoist():
    s_array = Int32MultiArray()
    s_array.data = [int(internal_vars['smoist'][0]), \
                    int(internal_vars['smoist'][1])]
    return s_array


sensor_funcs = {
    'cur'    : get_cur,
    'light'  : get_light,
    'level'  : get_level,
    'temp'   : get_temp,
    'humid'  : get_humid,
    'smoist' : get_smoist
}

now = int(time.time())
# Convert to midnight
now += clock_start - time_since_midnight(now)

try:
    baseline_schedule = baseline.Baseline(args.baseline, now)
    now = baseline_schedule.time0
except:
    print('baseline file %s not found or parse error' %args.baseline)
    exit()

rospy.init_node('Simulator', anonymous=True)

generate_publishers()
generate_subscribers()

#print("Now: %f %s" %(now, clock_time(now)))
clock_pub.publish(rospy.Time.from_sec(now))
last_update = now

while not rospy.core.is_shutdown():
    cur_interval = now - last_update
    last_update = now
    for f in update_funcs.values():
        f(cur_interval)

    #update sensors (calculations) + publish
    for sensor in sensor_names:
        publishers[sensor].publish(sensor_funcs[sensor]())
    if is_logging:
        print("==========================")
        print(now)
        print(internal_vars)
        print(actuator_vars)

    tick_interval = 1.0/actuator_vars['freq']
    # Don't use rospy.sleep here, because of speedup != 1
    time.sleep(tick_interval)

    old_speedup = speedup
    speedup = min(default_speedup,
                  (max_speedup_pump if actuator_vars['wpump'] else 100000),
                  (max_speedup_fans if actuator_vars['fan'] else 100000))
    if (is_logging and old_speedup != speedup): print("Speedup %d" %speedup)
    now += (tick_interval * speedup)
    clock_pub.publish(rospy.Time.from_sec(now))
    if baseline_schedule.update(now, internal_vars):
        # Need to do a funky thing converting wlevel to volume
        if (internal_vars.get('wlevel')):
            internal_vars['volume'] = internal_vars['wlevel']*volume_rate
            internal_vars.pop('wlevel')
        print(internal_vars)
