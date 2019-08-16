#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32,Bool,Float32,String,Int32MultiArray,Float32MultiArray
from topic_def import *
import argparse

parser = argparse.ArgumentParser(description = "simulator parser for Autonomous Systems")
parser.add_argument('baseline', type = str, default = "baseline.txt", nargs = "?")
args = parser.parse_args()
print(args.baseline)
try:
    exec(open(args.baseline).read())
except:
    print('no baseline file found')
    exit()

actuator_vars = init_actuators
internal_vars = init_internals
publishers = {}
subscribers = {}


###CONSTANTS
evap_rate = 0.1 #ml/s
flow_rate = 30.0 #ml/s
led_current = 3.2/255
pump_current = .2
fan_current = .06

def generate_publishers():
    global publishers
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
    for name in actuator_names:
        sub_name = name + "_raw"
        subscribers[name] = rospy.Subscriber(sub_name,
                                actuator_types[name],
                                generate_cb(actuator_vars, name))

def amb_light(time):
    return 0

### INTERNAL UPDATE FUNCTIONS ###
def light_update(cur_interval):
    for i in range(2):
        internal_vars['light'][i] = actuator_vars['led'] * 3 + amb_light(rospy.get_time())

def volume_update(cur_interval):
    internal_vars['volume'] -= cur_interval * flow_rate if actuator_vars['wpump'] else 0
    internal_vars['volume'] = max(0.01, internal_vars['volume'])
    
def humidity_update(cur_interval):
    for i in range(2):
        internal_vars['humidity'][i] += -cur_interval if bool(actuator_vars['fan']) else cur_interval
        internal_vars['humidity'][i] = max(0, internal_vars['humidity'][i])
        internal_vars['humidity'][i] = min(100, internal_vars['humidity'][i])

def temperature_update(cur_interval):
    for i in range(2):
        internal_vars['temperature'][i] = 1 if actuator_vars['fan'] else 0

def current_update(cur_interval):
    internal_vars['current'][0] = 512
    internal_vars['current'][0] += led_current * actuator_vars['led']
    internal_vars['current'][0] += pump_current if actuator_vars['wpump'] else 0 
    internal_vars['current'][0] += fan_current if actuator_vars['fan'] else 0
    
    internal_vars['current'][1] = 0.0 #power

def smoist_update(cur_interval):
    for i in range(2):
        internal_vars['smoist'][i] += cur_interval if actuator_vars['wpump'] else 0
        internal_vars['smoist'][i] -= cur_interval * evap_rate


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
    return float(internal_vars['volume'] / 20)

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
            publishers[sensor].publish(sensor_funcs[sensor]())
