#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32,Bool,Float32,String
from topic_def import sensor_names, actuator_names, to_ard, from_ard

values = {}

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
    values[name] = data.data

def generate_subscribers():
    global subscribers
    for name in actuator_names:
        sub_name = name + "_raw"
        cb = lambda data: update_request(name, data)
        subscribers[name] = rospy.Subscriber(sub_name, to_ard[name], cb)

def light_update(cur_interval):
    values['light'] = values['led']

def level_update(cur_interval):
    values['level'] = values['wpump']

def tds_update(cur_interval):
    values['tds'] = values['npump']

def hum_update(cur_interval):
    values['humid'] = values['level'] + values['fan']

def temp_update(cur_interval):
    values['temp'] = values['fan']

def cur_update(cur_interval):
    values['cur'] = 'idk'

rospy.init_node('Simulator', anonymous=True)
rospy.set_param("use_sim_time", True)
cur_time = 0

while  not rospy.core.is_shutdown():
    
    if rospy.get_time() - cur_time >= 1.0/values[freq]:

        cur_time = rospy.get_time()
        cur_interval = 1.0/values[freq]
        
        #update sensors (calculations) + publish
        for sensor in sensor_names:
            values[sensor] = globals()[sensor + '_update'](cur_interval)
            publishers[sensor].publish(values[sensor])


