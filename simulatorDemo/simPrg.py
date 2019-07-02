#!/usr/bin/env python3
import rospy
import rosbag
from std_msgs.msg import Int16, Bool
import csv

lightGiven = 0
pumpTime = 0 #min

pumpRate = 73 #cubicIn/min
pumpOn = False
waterAdded = 0

def lightCb(data):
    global lightGiven
    lightGiven = data.data

def waterCb(data):
    global pumpOn
    pumpOn = data.data 


#converts pump time to rise in water level
def waterConvert(time, rate):
    return time*rate/200

if __name__ == '__main__':


    rospy.init_node('Simulator', anonymous=True)

    lightPub = rospy.Publisher('lightHave', Int16, queue_size=10)
    lightSub = rospy.Subscriber('lightGive', Int16, lightCb)

    waterPub = rospy.Publisher('waterLevel', Int16, queue_size=10)
    waterSub = rospy.Subscriber('pumpTime', Bool, waterCb)

    rate = rospy.Rate(5)
    wait = rospy.Rate(3)
    while not rospy.is_shutdown():
        with open('testSim.csv') as csvfile:
            readCSV = csv.reader(csvfile, delimiter=',')
            for row in readCSV:

                rospy.loginfo("csv: %s lightStdGive: %s " % (row[1], lightGiven))
                lightPub.publish(int(row[1])+lightGiven)
               
                
                waterAdd = waterConvert(pumpTime, pumpRate)
                rospy.loginfo("csv: %s waterTimeGive: %s time2Level: %s " % (row[2], pumpTime, waterAdd))
                if pumpOn:
                    waterPub.publish(int(row[2])+ waterAdded + 3) #pumpRate/3600) #water given each sec
                    rate.sleep()
                    waterAdded+=3
                    pumpTime-=1

                wait.sleep()
        rate.sleep()



