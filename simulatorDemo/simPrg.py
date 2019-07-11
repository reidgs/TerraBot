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

#global vars
log_files = {}

sensor_names = ['tds', 'cur', 'light', 'level', 'temp', 'hum']
actuator_names = ['freq', 'led', 'wpump', 'npump', 'apump', 'fan']



time_range = args.time_range
time_freq = args.time_freq

light_given = 0.0

pump_rate = 73.0 #cubicIn/min
pump_on = False 
water_added = 0.0

#logging
def gen_log_files():
    global log_files
    prefix = time.strftime("%Y%m%d_%H%M%S_")
    os.makedirs("sim_log_%s" % prefix)

    for name in sensor_names + actuator_names:
        file_name = "sim_log_%s/%s.csv" % (prefix, name)
        log_files[name] = open(file_name, 'w+', 0)


def log_info(name, data):
    log_file = log_files[name]
    log_file.write(str(time.time()) + "," + str(data) + "\n")
    log_file.flush


#callbacks
def light_cb(data):
    global light_given
    light_given = float(data.data)

def water_cb(data):
    global pump_on
    pump_on = data.data


if __name__ == '__main__':

    rospy.init_node('Simulator', anonymous=True)
    
    #setting up pubs/subs
    light_pub = rospy.Publisher('light_output', Int32, queue_size=100)
    light_sub = rospy.Subscriber('led_input', Int32, light_cb)

    water_pub = rospy.Publisher('level_output', Int32, queue_size=100)
    water_sub = rospy.Subscriber('wpump_input', Bool, water_cb)

    ntr_pub = rospy.Publsiher('tds_output', Int32, queue_size=100)
    ntr_sub = rospy.Subscriber('npump_input', Bool, ntr_cb)

    hum_pub = rospy.Publisher('hum_output', Int32, queue_size=100)
    temp_pub = rospy.Publisher('temp_output', Int32, queue_size=100)
    fan_sub = rospy.Subscriber('fan_input', Bool, fan_cb)    
    
    time = rospy.Publisher('time', Int32, queue_size=10)
   
    gen_log_files()
   
    wait = rospy.Rate(time_freq)
    for time_now in range(time_range):
       
        
        #calculations
        if water_added > 0:
            #evaporation/usage of added water !!! LOL DONT ACTAULLY KNOW HOW MUCH THIS WILL BE...!!
            water_added -= 0.05*pumpRate/200.0 #fraction of water rise from pump...
        if pump_on:
            water_added += pump_rate/200.0
        
        #imported data from function
        light_d = d.light_data(time_now)
        water_d = d.water_data(time_now)

        #publishing
        light_pub.publish(light_d +light_given)
        water_pub.publish(water_d + water_added) #water given each sec
            
        sim_time.publish(time_now)
        
        #logging
        



        #rospy.loginfo("water data: %s pump: %s total: %s " % \
         #           (waterD, pumpOn, waterD + waterAdded))
       # rospy.loginfo("light data: %s lightStdGive: %s total: %s" % \
        #        (lightD, lightGiven, lightD+lightGiven))


        wait.sleep()

