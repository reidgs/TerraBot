import rospy
from std_msgs.msg import Int32,Bool,Float32,String
from topic_def import *

act_interf = {}
schedules = {}
next_update ={}

###interference functions###

def identity(x):
    return x

def off(x):
    return 0

def noise(x):
    return x+10


### default ###
for n in sensor_names + actuator_names:
    act_interf[n] = identity
    schedules[n] = {}
    next_update[n] = -1


def parse_interf(path=None):
    if path == None:
        return
    with open(path) as f:
        lines = [l.strip().split(",") for l in f.readlines()]
    lst = lines
    for l in lst:
        schedules[l[1]][l[0]] = l[2]
    for s in schedules:
        if len(schedules[s]) > 0:
            next_update[l[1]] = min(schedules[l[1]], key=int)

states_funcs = {
    'normal' : identity,
    'noise'  : noise,
    'off'    : off
}



### interf passthrough ###
def get_inter(name, time):
    if next_update[name] != -1 and \
            time.to_sec() >= int(next_update[name]):
        state = schedules[name].pop(next_update[name])
        next_update[name] = min(schedules[name], key=int) \
                if len(schedules[name]) > 0 else -1
        act_interf[name] = states_funcs[state]
    return act_interf[name]




###conversion functions###

#def light_inter(x):
 #   return int(x*3.41+13.531)  #converted to lux

#def cur_inter(x):
 #   return (x-512)*.0491 #converted to amps


