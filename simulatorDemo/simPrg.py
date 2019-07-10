#!/usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import Int32, Float32, Bool
import csv

lightGiven = 0

pumpRate = 73 #cubicIn/min
pumpOn = False 
waterAdded = 0

timeNow = 0
timeFreq = 1
freqGiven = False

def timeRateCb(data):
    global timeFreq
    global freqGiven
    timeFreq = data.data
    freqGiven = True

def lightCb(data):
    global lightGiven
    lightGiven = data.data

def waterCb(data):
    global pumpOn
    pumpOn = data.data

#converts pump time to rise in water level
def waterConvert(time, rate):
    return time*rate/200.0

if __name__ == '__main__':


    rospy.init_node('Simulator', anonymous=True)

    lightPub = rospy.Publisher('lightHave', Float32, queue_size=10)
    lightSub = rospy.Subscriber('lightGive', Float32, lightCb)

    waterPub = rospy.Publisher('waterLevel', Float32, queue_size=10)
    waterSub = rospy.Subscriber('pumpOn', Bool, waterCb)

    #simulated time
    simTime = rospy.Publisher('simTime', Int32, queue_size=10)
    timeRate = rospy.Subscriber('timeRate', Int32, timeRateCb)
    
    if not rospy.is_shutdown():
        while not freqGiven:
            pass
        wait = rospy.Rate(timeFreq)
        csvfile = open('new.csv')
        readCSV = csv.reader(csvfile, delimiter=',')

        for row in readCSV:
            if waterAdded > 0:
                waterAdded-=1/3.0 #evaporation/usage of added water
            if pumpOn:
                waterAdded += 2 #waterConvert(1.0/60, pumpRate) #assumes 1 msg/sec
            
            rospy.loginfo("water csv: %s pump: %s total: %s " % \
                        (row[2], pumpOn, float(row[2]) + waterAdded))
            rospy.loginfo("light csv: %s lightStdGive: %s total: %s" % \
                        (row[1], lightGiven, float(row[1])+lightGiven))
            lightPub.publish(float(row[1])+lightGiven)
            waterPub.publish(float(row[2]) + waterAdded) #water given each sec
            
            timeNow += 1
            simTime.publish(timeNow)
            wait.sleep()

        csvfile.close()
