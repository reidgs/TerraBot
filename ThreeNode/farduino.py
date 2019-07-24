#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32,Bool,Float32,String
from topic_def import *

values = {}
publishers = {}
subscribers = {}

def generate_values():
    global values
    for name in actuator_names + sensor_names:
        values[name] = 0

def generate_publishers():
    global publishers
    for name in sensor_names:
        pub_name = name + "_raw"
        publishers[name] = rospy.Publisher(
                            pub_name, from_ard[name],
                            latch = True, queue_size = 100)

#used when generating subs
def update_request(name, data):
    global values
    print (name)
    values[name] = data.data

def generate_cb(name):
    return (lambda data: update_request(name,data))

def generate_subscribers():
    global subscribers
    for name in actuator_names:
        sub_name = name + "_raw"

        subscribers[name] = rospy.Subscriber(sub_name, to_ard[name], generate_cb(name))

def light_update(cur_interval):
    values['light'] = values['led']

def level_update(cur_interval):
    values['level'] = 1 if values['wpump'] else 0

def tds_update(cur_interval):
    values['tds'] = 1 if values['npump'] else 0

def humid_update(cur_interval):
    values['humid'] = 1 if values['fan'] else 0

def temp_update(cur_interval):
    values['temp'] = 1 if values['fan'] else 0

def cur_update(cur_interval):
    values['cur'] = 1 if values['led'] else 0

rospy.set_param("use_sim_time", True)
rospy.init_node('Simulator', anonymous=True)
cur_time = 0

generate_values()
generate_publishers()
generate_subscribers()
values['freq'] = 1

for s in subscribers:
    subscribers[s].callback(Int32(12))
while  not rospy.core.is_shutdown():

    if rospy.get_time() - cur_time >= 1.0/values['freq']:

        cur_time = rospy.get_time()
        cur_interval = 1.0/values['freq']
        print([(k,values[k]) for k in values.keys()])
        #update sensors (calculations) + publish
        for sensor in sensor_names:
            values[sensor] = globals()[sensor + '_update'](cur_interval)
            publishers[sensor].publish(values[sensor])


