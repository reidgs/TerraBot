import rospy
from std_msgs.msg import Int32,Bool,Float32,String,Int32MultiArray,Float32MultiArray
from numpy.random import normal
from datetime import datetime
from terrabot_utils import clock_to_seconds, clock_time, time_since_midnight
from terrabot_utils import Agenda
from topic_def import *

# If string is a number, return that number, o/w return the string
def floatify (f, name):
    if (f.find('prop') > 0):
        return ('prop', float(f.strip('()').split(' ')[1]))
    else:
        try:
            return types[name](float(f))
        except ValueError:
            return f

types = {
    
    'led'   : int,
    'wpump' : bool,
    'fan'   : bool,
    'freq'  : float,

    #type for each sensor value
    'smoist' : int,
    'cur'    : float,
    'light'  : int,
    'level'  : float,
    'temp'   : int,
    'humid'  : int,
}

std_dev = { 'led'   : 0,
            'wpump' : 0,
            'fan'   : 0,
            'smoist' : 10,
            'cur'    : 1,
            'light'  : 5,
            'level'  : 2,
            'temp'   : 1,
            'humid'  : 2,
            }

proportionality = { 'led'   : 1.0,
            'wpump' : 1.0,
            'fan'   : 1.0,
            'smoist' : 1.0,
            'cur'    : 1.0,
            'light'  : 1.0,
            'level'  : 1.0,
            'temp'   : 1.0,
            'humid'  : 1.0,
            }

name_translations = { 'led' : 'led',
                      'wpump' : 'wpump',
                      'fan' : 'fan',
                      'smoist' : 'smoist',
                      'current' : 'cur',
                      'light' : 'light',
                      'wlevel' : 'level',
                      'temperature' : 'temp',
                      'humidity' : 'humid'}

###interference functions###

def identity(name, x):
    return x

def off(name, x):
    return types[name](0)

def on(name, x):
    return types[name](1)

def noise(name, x):
    return types[name](normal(x, std_dev[name]))

def proportional(name, x):
    return types[name](x * proportionality[name])

states_funcs = {
    'normal' : identity,
    'noise'  : noise,
    'off'    : off,
    'on'     : on,
    'prop'   : proportional
}

def get_interf_funcs(value):
    if (type(value) is str):
        return states_funcs[value]
    elif (type(value) is tuple and value[0] == 'prop'):
        return lambda name, x: types[name_translations.get(name)](x*value[1])
    else:
        return lambda name, x: value

class Interference(Agenda):
    interf_funcs = {}
    def __init__(self, filename, time0):
        for n in sensor_names:
            self.interf_funcs[n] = (identity if n == 'level' else
                                    [identity, identity])
        for n in actuator_names:
            self.interf_funcs[n] = identity

        if (not filename): return # Just use the defaults/identities
        self.time0 = time0
        last_time = time0
        with open(filename) as f:
            for line in f.readlines():
                l = line.split('#')[0].strip(' \n')
                if (l.find('AT') == 0):
                    dtime = datetime.strptime(l, "AT %d-%H:%M:%S")
                    time = time0 + clock_to_seconds(dtime)
                    if (time < last_time):
                        print("Time must run forward: %s" %l)
                        quit()
                    last_time = time
                    interfs = []
                    self.add_to_schedule([time, interfs])
                elif (len(l) > 0):
                    interf = l.split("=")
                    if (len(interf) != 2):
                        print("Illegal syntax: %s" %l); quit()

                    interf_name = interf[0].strip()
                    topic_name = name_translations.get(interf_name)
                    if (not topic_name):
                        print("%s not a legal interference sensor name"
                              %interf_name)
                        quit()
                    interf_val = interf[1]
                    if (interf_val.find('[') >= 0):
                        interf_val = [floatify(iv.strip(' []'), topic_name)
                                      for iv in interf_val.split(',')]
                    else: 
                        interf_val = floatify(interf_val.strip(), topic_name)
                    interfs.append([topic_name, interf_val])

    def update(self, time):
        if (not self.finished() and (time >= self.schedule[self.index][0])):
            print("Updating interferences at %s" %clock_time(time))
            for ifs in self.schedule[self.index][1]:
                if (type(ifs[1]) == list):
                    funcs = [get_interf_funcs(i) for i in ifs[1]]
                else:
                    funcs = get_interf_funcs(ifs[1])
                self.interf_funcs[ifs[0]] = funcs
            self.index += 1
            return True
        else: return False

    # Get the interference functions
    def edit(self, name, value):
        if (type(value) is list or type(value) is tuple):
            return (self.interf_funcs[name][0](name, value[0]),
                    self.interf_funcs[name][1](name, value[1]))
        else:
            return self.interf_funcs[name](name, value)

    def display(self):
        for interf in self.schedule:
            print("AT %s" %clock_time(interf[0]))
            for iv in interf[1]:
                print("  %s = %s" %(iv[0], iv[1]))

if __name__ == '__main__':
    def p(n,v,t):
        print("%s: %s" %(n, interference.edit(n,v)))

    import sys, time
    if (len(sys.argv) == 2):
        now = time.time()
        time0 = now - time_since_midnight(now)
        interference = Interference(sys.argv[1], time0)
        interference.display()
        #"""
        # Testing how interference works
        t = interference.time0; sensor_values = {}
        while not interference.finished():
            if (interference.update(t)):
                p('humid', [60, 60], t)
                p('temp', [30, 30], t)
                p('smoist', [450, 450], t)
                p('light', [350, 350], t)
                p('level', 125.3, t)
                p('cur', [52.0, 1000], t)
                p('fan', True, t)
                p('wpump', True, t)
                p('led', 200, t)
            t += 1
        #"""
    else:
        print("Need to provide interference file to parse")
