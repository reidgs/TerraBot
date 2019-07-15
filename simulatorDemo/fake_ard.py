#!/usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import Int32, Float32, Bool
from data import *
import argparse
import time as t
import os

#flags for timeRange and timeFreq
parser = argparse.ArgumentParser()
parser.add_argument('time_range', type=int)
parser.add_argument('time_freq', type=int)
args = parser.parse_args()

time_range = args.time_range
time_freq = args.time_freq

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
def tds_update(secs):
    global ntr_added, npump_on, npump_rate, tds_now, time_now
    if npump_on:
        ntr_added += secs*npump_rate
    tds_now = tds_data(time_now) + ntr_added #this is volume rn!! need to convert to tds!!

def level_update(secs):
    global water_added, wpump_on, wpump_rate, level_now, time_now
    if water_added > 0:
    #evaporation/usage of added water !!!DONT ACTAULLY KNOW HOW MUCH THIS WILL BE...!!
        water_added -= 0.05*secs*wpump_rate/200.0 #fraction of water rise from pump...for now
    if wpump_on:
        water_added += secs*wpump_rate/200.0
    #imported + std added data
    level_now = light_data(time_now) + water_added

def hum_update():
    global hum_now, time_now
    #how do i even relate this to fans + water pump

def temp_update():
    global temp_now, time_now
    #prob don't need to incorporate fans
    temp_now = temp_data(time_now)

def cur_update():
    global cur_now, time_now
    #um somehow add up cur used by actuators...
    cur_now = cur_data(time_now)



if __name__ == '__main__':

    rospy.init_node('Simulator', anonymous=True)

    #setting up pubs/subs & log files & rate
    light_pub = rospy.Publisher('light_raw', Int32, queue_size=100)
    led_sub = rospy.Subscriber('led_raw', Int32, led_cb)

    level_pub = rospy.Publisher('level_raw', Int32, queue_size=100)
    wpump_sub = rospy.Subscriber('wpump_raw', Bool, wpump_cb)

    tds_pub = rospy.Publisher('tds_raw', Int32, queue_size=100)
    npump_sub = rospy.Subscriber('npump_raw', Bool, npump_cb)

    apump_sub = rospy.Subscriber('apump_raw', Bool, apump_cb)

    hum_pub = rospy.Publisher('humid_raw', Int32, queue_size=100)
    temp_pub = rospy.Publisher('temp_raw', Int32, queue_size=100)
    fan_sub = rospy.Subscriber('fan_raw', Bool, fan_cb)

    time_pub = rospy.Publisher('time_raw', Int32, queue_size=100)
    freq_sub = rospy.Subscriber('freq_raw', Float32, freq_cb)

    cur_pub = rospy.Publisher('cur_raw', Float32, queue_size = 100)

    wait = rospy.Rate(time_freq)

    if pub_freq < 1:
        freq = 1
        x = round(1.0/pub_freq)
    else:
        freq = pub_freq

    for sec in range(time_range):
        if pub_freq < 1 and sec % x != 0:
                continue

        for cycle in range(freq):

            time_now = sec + cycle/pub_freq

            #update sensors
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
            time_pub.publish(time_now)

        wait.sleep()










