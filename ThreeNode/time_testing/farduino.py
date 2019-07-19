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
        pub_name = name + "_output"
        publishers[name] = rospy.Publisher(
                            pub_name, from_ard[name],
                            latch = True, queue_size = 100)

def update_request(name, data):
    values[name] = data.data

def generate_subscribers():
    global subscribers
    for name in actuator_names:
        sub_name = name + "_input"
        cb = lambda data: update_request(name, data)
        subscribers[name] = rospy.Subscriber(sub_name, to_ard[name], cb)


time_update = {}

def nothing(data):
    True

for n in sensor_names + actuator_names:
    callback_dict[n] = nothing

def light_update():
    values['light'] = values['led']

time_update['light'] = light_update


current_time = 0
interval = 1
def time_cb(data):
    if data.data - current_time >= interval:
        for name in actuator_names:
            values[name] = requests[name]
        for name in sensor_names:
            publishers[name].publish(values[name])
