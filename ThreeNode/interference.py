import rospy
from std_msgs.msg import Int32,Bool,Float32,String
from topic_def import sensor_names, actuator_names, pub_types, sub_types

import subprocess, time


#interference functions

interf_dict = {}

def identity(x):
    return x

def light_inter(x):
    return int(x*3.41+13.531)  #converted to lux

def cur_inter(x):
    return (x-512)*.0491 #converted to amps

def cam_inter(x):
    if x:
        time_stamp = "Photos/" + str(time.time()) + ".jpg"
        subprocess.call("raspistill -n -md 2 -awb off -awbg 1,1 -ss 30000 -o %s" % time_stamp, shell = True)
        return time_stamp
    else:
        return "there was an error"
        

for n in sensor_names + actuator_names:
    interf_dict[n] = identity

interf_dict['light'] = light_inter
interf_dict['cur'] = cur_inter
interf_dict['cam'] = cam_inter

def get_inter(name):
    return interf_dict[name]

    
