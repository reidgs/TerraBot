import rospy
from std_msgs.msg import Int32,Bool,Float32,String,Int32MultiArray,Float32MultiArray
from topic_def import *

interf_funcs = {}
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
    interf_funcs[n] = identity
    schedules[n] = {}
    next_update[n] = -1


def parse_interf(path=None):
    if path == None:
        return
    with open(path) as f:
        lines = [l.strip().split(",") for l in f.readlines()]
    lst = lines
    for l in lst:
        #redundant sensors
        if l[1] != 'level' and l[1] in sensor_names:
            schedules[l[1]][l[0]] = [l[2],l[3]]
        else:
            schedules[l[1]][l[0]] = l[2]
    for s in schedules:
        if len(schedules[s]) > 0:
            next_update[s] = min(schedules[s], key=int)

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
        
        #redundant sensors
        if type(state) == list:
            temp = []
            for x in state:
                temp.append(states_funcs[x])
            interf_funcs[name] = temp

        else:
            interf_funcs[name] = states_funcs[state]

    return interf_funcs[name]




###conversion functions###

#def light_inter(x):
 #   return int(x*3.41+13.531)  #converted to lux

#def cur_inter(x):
 #   return (x-512)*.0491 #converted to amps


