#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32,Bool,Float32,String
from topic_def import *
import argparse

parser = argparse.ArgumentParser(description = "simulator parser for Autonomous Systems")
parser.add_argument('baseline', type = str, default = "baseline.py", nargs = "?")
args = parser.parse_args()
print(args.baseline)
try:
    exec(open(args.baseline).read())
except:
    print('no baseline file found')
    exit()

actuator_vars = init_actuators
internal_vars = init_internals
print(actuator_vars)
publishers = {}
subscribers = {}

farduino_types = {
    'led'   : int,
    'wpump' : bool,
    'npump' : bool,
    'apump' : bool,
    'fan'   : bool,
    'freq'  : float,
    'tds'   : int,
    'cur'   : int,
    'light' : int,
    'level' : float,
    'temp'  : int,
    'humid' : int,
}


###CONSTANTS
evap_rate = 0.1 #ml/s
flow_rate = 30.0 #ml/s
led_current = 0.25
pump_current = 10
fan_current = 5

def generate_publishers():
    global publishers
    for name in sensor_names:
        pub_name = name + "_raw"
        publishers[name] = rospy.Publisher(
                            pub_name, from_ard[name],
                            latch = True, queue_size = 100)

#used when generating subs
def update_keyval(dictionary, name, data):
    dictionary[name] = data.data

def generate_cb(dictionary, name):
    return (lambda data: update_keyval(dictionary,name,data))

def generate_subscribers():
    global subscribers
    for name in actuator_names:
        sub_name = name + "_raw"
        subscribers[name] = rospy.Subscriber(sub_name,
                                to_ard[name],
                                generate_cb(actuator_vars, name))

def amb_light(time):
    return 0

### INTERNAL UPDATE FUNCTIONS ###
def light_update(cur_interval):
    internal_vars['total_light'] = actuator_vars['led'] * 3 + amb_light(rospy.get_time())

def volume_update(cur_interval):
    internal_vars['volume'] += cur_interval * flow_rate if actuator_vars['wpump'] else 0
    internal_vars['volume'] -= evap_rate * cur_interval
    internal_vars['volume'] = max(0.01, internal_vars['volume'])

def nutrient_update(cur_interval):
    internal_vars['nutrient'] += (cur_interval * flow_rate) if actuator_vars['npump'] else 0
    #convert amt of ntr to tds (consider water vol)

def humidity_update(cur_interval):
    internal_vars['humidity'] += -cur_interval if bool(actuator_vars['fan']) else cur_interval
    internal_vars['humidity'] = max(0, internal_vars['humidity'])
    internal_vars['humidity'] = min(100, internal_vars['humidity'])

def temperature_update(cur_interval):
    internal_vars['temperature'] = 1 if actuator_vars['fan'] else 0

def current_update(cur_interval):
    internal_vars['current'] = 512
    internal_vars['current'] += led_current * actuator_vars['led']
    internal_vars['current'] += pump_current if actuator_vars['wpump'] else 0
    internal_vars['current'] += pump_current if actuator_vars['npump'] else 0
    internal_vars['current'] += pump_current if actuator_vars['apump'] else 0
    internal_vars['current'] += fan_current if actuator_vars['fan'] else 0

update_funcs = {
    'volume'   : volume_update,
    'nutrient' : nutrient_update,
    'total_light' : light_update,
    'temperature' : temperature_update,
    'humidity'  : humidity_update,
    'current' : current_update
}

### INTERNAL TO SENSOR TRANSLATION ###
def get_tds():
    return internal_vars['nutrient'] / internal_vars['volume']

def get_cur():
    return internal_vars['current']

def get_light():
    return internal_vars['total_light']

def get_level():
    return internal_vars['volume'] / 20

def get_temp():
    return internal_vars['temperature']

def get_humid():
    return internal_vars['humidity']

sensor_funcs = {
    'tds'   : get_tds,
    'cur'   : get_cur,
    'light' : get_light,
    'level' : get_level,
    'temp'  : get_temp,
    'humid'   : get_humid,
}

rospy.init_node('Simulator', anonymous=True)

generate_publishers()
generate_subscribers()
last_pub = rospy.get_time()
last_update = rospy.get_time()

while not rospy.core.is_shutdown():
    now = rospy.get_time()
    cur_interval = now - last_update
    last_update = now
    for f in update_funcs.values():
        f(cur_interval)

    if rospy.get_time() - last_pub >= 1.0/actuator_vars['freq']:
        last_pub = rospy.get_time()
        #update sensors (calculations) + publish
        print("==========================")
        print(rospy.get_time())
        print(internal_vars)
        print(actuator_vars)
        for sensor in sensor_names:
            publishers[sensor].publish(
                farduino_types[sensor](
                    sensor_funcs[sensor]())
                )
