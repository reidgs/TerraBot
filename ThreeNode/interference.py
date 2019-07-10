import rospy
from std_msgs.msg import Int32,Bool,Float32,String
import subprocess
    

sensor_names = ['tds', 'cur', 'light', 'level', 'temp', 'hum']
actuator_names = ['freq', 'led', 'wpump', 'npump', 'apump', 'fan']

pub_types = {
    'tds'   : Int32,
    'cur'   : Float32,
    'light' : Int32,
    'level' : Int32,
    'temp'  : Int32,
    'hum'   : Int32,
    'freq'  : Float32,
    'led'   : Int32,
    'wpump' : Bool,
    'npump' : Bool,
    'apump' : Bool,
    'fan'   : Bool,
    'cam'   : String
}

sub_types = {
    'tds'   : Int32,
    'cur'   : Int32,
    'light' : Int32,
    'level' : Int32,
    'temp'  : Int32,
    'hum'   : Int32,
    'freq'  : Float32,
    'led'   : Int32,
    'wpump' : Bool,
    'npump' : Bool,
    'apump' : Bool,
    'fan'   : Bool,
    'cam'   : Bool
}

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
        time_stamp = time.time()
        subprocess.call("raspistill -o /Photos/%s" % time_stamp, shell = True)
        return time_stamp
    else:
        return None
        

for n in sensor_names + actuator_names:
    interf_dict[n] = identity

interf_dict['light'] = light_inter
interf_dict['cur'] = cur_inter
interf_dict['cam'] = cam_inter

def get_inter(name):
    return interf_dict[name]

    
