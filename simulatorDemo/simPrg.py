#!/usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import Float32, Bool
import csv

lightGiven = 0

pumpRate = 73 #cubicIn/min
pumpOn = False 
waterAdded = 0

def lightCb(data):
    global lightGiven
    lightGiven = data.data

def waterCb(data):
    global pumpOn
    global waterAdded
    pumpOn = data.data
    if waterAdded > 0:
        waterAdded-=1/5.0 #evaporation/usage of added water
    rospy.loginfo(pumpOn)
    if pumpOn:
        waterAdded += 2 #waterConvert(1.0/60, pumpRate) #assumes 1 msg/sec


#converts pump time to rise in water level
def waterConvert(time, rate):
    return time*rate/200.0

if __name__ == '__main__':


    rospy.init_node('Simulator', anonymous=True)

    lightPub = rospy.Publisher('lightHave', Float32, queue_size=10)
    lightSub = rospy.Subscriber('lightGive', Float32, lightCb)

    waterPub = rospy.Publisher('waterLevel', Float32, queue_size=10)
    waterSub = rospy.Subscriber('pumpOn', Bool, waterCb)

    rate = rospy.Rate(5)
    wait = rospy.Rate(3)
    while not rospy.is_shutdown():
        with open('new.csv') as csvfile:
            readCSV = csv.reader(csvfile, delimiter=',')
            for row in readCSV:

                rospy.loginfo("csv: %s lightStdGive: %s " % (row[1], lightGiven))
                lightPub.publish(float(row[1])+lightGiven)
                
                waterPub.publish(float(row[2]) + waterAdded) #water given each sec
                rate.sleep()
                rospy.loginfo("csv: %s total: %s " % (row[2], waterAdded))
                wait.sleep()
        rate.sleep()



