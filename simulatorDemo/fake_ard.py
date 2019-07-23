#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Float32, Bool

#initialization of vars 
pub_freq = 1
interval = 1.0/pub_freq # secs for interval btwn publishing

time_now = 0.0

led_given = 0.0
light_now = 0.0

wpump_rate = 1.83 #cubicIn/s
wpump_on = False
level_now = 50.0 #height(in) --initial tank amt

npump_rate = 1.83 #cubicIn/s
npump_on = False
ntr_added = 0.0 #volume
tds_now = 0

fan_on = False
hum_now = 0
temp_now = 50

cur_now = 10 

rospy.init_node('Simulator', anonymous=True)



#callbacks
def time_cb(data):
    global time_now
    if data.data - time_now >= interval:
        time_now = data.data
        
        #update sensors after calculations
        light_update()
        level_update()
        tds_update()
        hum_update()
        temp_update()
        cur_update()

        #publishing
        light_pub.publish(light_now)
        level_pub.publish(level_now)
        tds_pub.publish(tds_now)
        hum_pub.publish(hum_now)
        temp_pub.publish(temp_now)
        cur_pub.publish(cur_now)


def led_cb(data):
    global led_given
    led_given = data.data

def wpump_cb(data):
    global wpump_on
    wpump_on = data.data

def npump_cb(data):
    global npump_on
    npump_on = data.data

def fan_cb(data):
    global fan_on
    fan_on = data.data

def freq_cb(data):
    global pub_freq
    pub_freq = data.data

#calculations to update sensors

def light_update():
    global led_given, light_now
    light_now = led_given

def tds_update():
    global ntr_added, npump_on, npump_rate, tds_now, interval
    if npump_on:
        ntr_added += interval*npump_rate
    tds_now = ntr_added #this is volume rn!! need to convert to tds!!

def level_update():
    global wpump_on, wpump_rate, level_now, interval
    if level_now > 0:
        #water used, for now: fraction of pump rate
        level_now -= 0.05*interval*wpump_rate/200.0 
    if wpump_on:
        level_now += 100*interval*wpump_rate/200.0


def hum_update():
    global hum_now, light_now, wpump_on, fan_on
    #how do i even relate this to fans + water pump
    if light_now > 40 or wpump_on or fan_on:
        hum_now += 1

def temp_update():
    global temp_now
    #fans neccesary..?
    if fan_on:
        temp_now -= 1

def cur_update():
    global cur_now
    #um somehow add up cur used by actuators...
 

#setting up pubs/subs
light_pub = rospy.Publisher('light_raw', Int32, queue_size=100)
led_sub = rospy.Subscriber('led_raw', Int32, led_cb)

level_pub = rospy.Publisher('level_raw', Int32, queue_size=100)
wpump_sub = rospy.Subscriber('wpump_raw', Bool, wpump_cb)

tds_pub = rospy.Publisher('tds_raw', Int32, queue_size=100)
npump_sub = rospy.Subscriber('npump_raw', Bool, npump_cb)

#apump_sub = rospy.Subscriber('apump_raw', Bool, apump_cb)

hum_pub = rospy.Publisher('hum_raw', Int32, queue_size=100)
temp_pub = rospy.Publisher('temp_raw', Int32, queue_size=100)
fan_sub = rospy.Subscriber('fan_raw', Bool, fan_cb)

time_sub = rospy.Subscriber('time', Float32, time_cb)
freq_sub = rospy.Subscriber('freq_raw', Float32, freq_cb)

cur_pub = rospy.Publisher('cur_raw', Int32, queue_size = 100)

rospy.spin()
