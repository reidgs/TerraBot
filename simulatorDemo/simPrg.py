#!/usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import Int32, Float32, Bool
import data as d
import argparse


#flags for timeRange and timeFreq
parser = argparse.ArgumentParser()
parser.add_argument('timeRange', type=int)
parser.add_argument('timeFreq', type=int)
args = parser.parse_args()

#global vars
timeRange = args.timeRange
timeFreq = args.timeFreq

lightGiven = 0.0

pumpRate = 73.0 #cubicIn/min
pumpOn = False 
waterAdded = 0.0


#callbacks
def lightCb(data):
    global lightGiven
    lightGiven = float(data.data)

def waterCb(data):
    global pumpOn
    pumpOn = data.data


#math

#converts pump time to rise in water level
def waterConvert(time, rate):
    return time*rate/200.0

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
            waterAdded-=1/3.0 #evaporation/usage of added water
        if pumpOn:
            waterAdded += 2.0 #waterConvert(1.0/60, pumpRate) #assumes 1 msg/sec
        
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

