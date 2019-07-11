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
parser.add_argument('timeRange', type=int)
parser.add_argument('timeFreq', type=int)
args = parser.parse_args()

#global vars
log_files = {}

sensor_names = ['tds', 'cur', 'light', 'level', 'temp', 'hum']
actuator_names = ['freq', 'led', 'wpump', 'npump', 'apump', 'fan']



timeRange = args.timeRange
timeFreq = args.timeFreq

lightGiven = 0.0

pumpRate = 73.0 #cubicIn/min
pumpOn = False 
waterAdded = 0.0

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
#def lightCb(data):
 #   global lightGiven
  #  lightGiven = float(data.data)

#def waterCb(data):
 #   global pumpOn
  #  pumpOn = data.data


#math

    global pumpRate
    return time*pumpRate/200.0

if __name__ == '__main__':

    rospy.init_node('Simulator', anonymous=True)
    
    #setting up pubs/subs
    lightPub = rospy.Publisher('lightHave', Float32, queue_size=10)
    lightSub = rospy.Subscriber('lightGive', Float32, lightCb)

    waterPub = rospy.Publisher('waterLevel', Float32, queue_size=10)
    waterSub = rospy.Subscriber('pumpOn', Bool, waterCb)

    #simulated time
    simTime = rospy.Publisher('simTime', Int32, queue_size=10)

    wait = rospy.Rate(timeFreq)
    print(timeFreq)
    print(timeRange)
    for timeNow in range(timeRange):
       
        
        #calculations
        if waterAdded > 0:
            #evaporation/usage of added water !!! LOL DONT ACTAULLY KNOW HOW MUCH THIS WILL BE...!!
            waterAdded -= 0.05*pumpRate/200.0 #fraction of water rise from pump...
        if pumpOn:
            waterAdded += pumpRate/200.0
        
        #imported data from function
        lightD = d.lightData(timeNow)
        waterD = d.waterData(timeNow)

        #publishing
        lightPub.publish(lightD +lightGiven)
        waterPub.publish(waterD + waterAdded) #water given each sec
            
        simTime.publish(timeNow)
        
        #logging
        



        rospy.loginfo("water data: %s pump: %s total: %s " % \
                    (waterD, pumpOn, waterD + waterAdded))
        rospy.loginfo("light data: %s lightStdGive: %s total: %s" % \
                (lightD, lightGiven, lightD+lightGiven))


        wait.sleep()

