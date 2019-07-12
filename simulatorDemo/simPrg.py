#!/usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import Int32, Float32, Bool
import data as d
import argparse
import time
import os

#flags for timeRange and timeFreq
parser = argparse.ArgumentParser()
parser.add_argument('time_range', type=int)
parser.add_argument('time_freq', type=int)
args = parser.parse_args()

time_range = args.time_range
time_freq = args.time_freq

log_files = {}
sensor_names = ['tds', 'cur', 'light', 'level', 'temp', 'hum']
actuator_names = ['freq', 'led', 'wpump', 'npump', 'fan']

#initialization of vars 
pub_freq = 1

time_now = 0

led_given = 0.0
light_now = 0

wpump_rate = 1.83 #cubicIn/s
wpump_on = False 
water_added = 0.0 #height (in)
level_now = 0

npump_rate = 1.83 #cubicIn/s
npump_on = False
ntr_added = 0.0 #volume
tds_now = 0

fan_on = False
hum_now = 0.0
temp_now = 50

cur_now = 0.0

#logging
def gen_log_files():
    global log_files
    prefix = time.strftime("%Y%m%d_%H%M%S_")
    os.makedirs("sim_log_%s" % prefix)

    for name in sensor_names + actuator_names:
        file_name = "sim_log_%s/%s.csv" % (prefix, name)
        log_files[name] = open(file_name, 'w+', 0)


def log_info(name, data):
    f = globals()["d." + name + "_data"]
    t = time_now
    log_file = log_files[name]
    baseline = f(t) 
    log_file.write(t + "," + str(baseline) + \
                       "," +  str(data) + "\n")
    log_file.flush


#callbacks
def led_cb(data):
    global led_given
    led_given = float(data.data)
    log_info('led', led_given)

def wpump_cb(data):
    global wpump_on
    wpump_on = data.data
    log_info('wpump', wpump_on)

def npump_cb(data):
    global npump_on
    npump_on = data.data
    log_info('npump', npump_on)

def fan_cb(data):
    global fan_on
    fan_on = data.data
    log_info('fan', fan_on)

def freq_cb(data):
    global pub_freq
    pub_freq = data.data
    log_info('freq', pub_freq)


#calculations to update sensors 
def light_update():
    global light_now
    light_now = _given + d.light_data(time_now)

def tds_update(secs):
    global ntr_added, npump_on, npump_rate, tds_now
    if npump_on:
        ntr_added += secs*npump_rate
    tds_now = d.tds_data(time_now) + ntr_added #this is volume rn!! need to convert to tds!!

def level_update(secs):
    global water_added, wpump_on, wpump_rate, level_now
    if water_added > 0:
    #evaporation/usage of added water !!!DONT ACTAULLY KNOW HOW MUCH THIS WILL BE...!!
    water_added -= 0.05*secs*wpump_rate/200.0 #fraction of water rise from pump...for now
    if wpump_on:
        water_added += secs*wpump_rate/200.0
    #imported + std added data
    level_now = d.light_data(time_now) + water_added

def hum_update():
    global hum_now
    #how do i even relate this to fans + water pump
 
def temp_update():
    global temp_now
    #prob don't need to incorporate fans
    temp_now = d.temp_data(time_now)

def cur_update():
    global cur_now
    #um somehow add up cur used by actuators...

if __name__ == '__main__':

    rospy.init_node('Simulator', anonymous=True)
    
    #setting up pubs/subs & log files & rate
    light_pub = rospy.Publisher('light_output', Int32, queue_size=100)
    led_sub = rospy.Subscriber('led_input', Int32, led_cb)

    level_pub = rospy.Publisher('level_output', Int32, queue_size=100)
    wpump_sub = rospy.Subscriber('wpump_input', Bool, wpump_cb)

    tds_pub = rospy.Publsiher('tds_output', Int32, queue_size=100)
    npump_sub = rospy.Subscriber('npump_input', Bool, npump_cb)

    hum_pub = rospy.Publisher('hum_output', Int32, queue_size=100)
    temp_pub = rospy.Publisher('temp_output', Int32, queue_size=100)
    fan_sub = rospy.Subscriber('fan_input', Bool, fan_cb)    
    
    time = rospy.Publisher('time', Int32, queue_size=10)
    pub_freq = rospy.Subscriber('freq', Float32, freq_cb)

    gen_log_files()
   
    wait = rospy.Rate(time_freq)

    if pub_freq < 1:
        freq = 1
    else:
        freq = pub_freq

    for sec in range(time_range):
        if pub_freq < 1:
            x = int(1.0/pub_freq)
            if sec % x != 0:
                continue
            
        for cycle in range(freq):
            
            time_now = sec + cycle/pub_freq
            
            #update sensors
            light_update()
            level_update(1.0/pub_freq)
            tds_update(1.0/pub_freq)
            temp_update()
            cur_update()

            #publishing
            light_pub.publish(light_now) 
            level_pub.publish(level_now)
            tds_pub.publish(tds_now)
            hum_pub.publish(hum_now)
            temp_pub.publish(temp_now)
            cur_pub.publish(cur_now)
            sim_time.publish(time_now)
        
            #logging
            for sensor in sensor_names:
                log_info(sensor, globals()[sensor + '_now'])

        wait.sleep()

