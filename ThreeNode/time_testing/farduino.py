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

def update_request(name, data):
    values[name] = data.data

def generate_subscribers():
    global subscribers
    for name in actuator_names:
        sub_name = name + "_raw"
        cb = lambda data: update_request(name, data)
        subscribers[name] = rospy.Subscriber(sub_name, to_ard[name], cb)


#time_update = {}

#def nothing(data):
 #   True

#what is this
#for n in sensor_names + actuator_names:
 #   callback_dict[n] = nothing

def light_update():
    values['light'] = values['led']

def level_update():
    values['level'] = values['wpump']

def tds_update():
    values['tds'] = values['npump']

def hum_update():
    values['humid'] = values['level'] + values['fan']

def temp_update():
    values['temp'] = values['fan']

def cur_update():
    values['cur'] = 'idk'

#what is this
#time_update['light'] = light_update

#how to set /sim_time parameter..?
interval = 1
while  not rospy.core.is_shutdown():
    if rospy.get_time() - current_time >= interval:
        #update sensors (calculations)
        for name in actuator_names:
            values[name] = globals()[name + '_update']()
        #publishing sensor data
        for name in sensor_names:
            publishers[name].publish(values[name])
