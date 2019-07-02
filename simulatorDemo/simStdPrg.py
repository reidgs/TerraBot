#!/usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import Int16, Bool
import csv

lightSen = 0
curLight = 0
lightDesire = 500

waterLevel = 50
pumpRate = 73 #cubicIn/min
waterDesire = 50 #mm

def lightCb(data):
    global lightSen
    lightSen = data.data
    rospy.loginfo("light sensor: %s", data.data)

def waterCb(data):
    global waterLevel
    waterLevel = data.data
    rospy.loginfo("water Level: %s", data.data)

def level2time(rate, level):
    return level*200/rate 

if __name__ == '__main__':

    rospy.init_node('student', anonymous=True)
    lightPub = rospy.Publisher('lightGive', Int16, queue_size=10)
    lightSub = rospy.Subscriber('lightHave', Int16, lightCb)

    waterPub = rospy.Publisher('pumpOn', Bool, queue_size=10)
    waterSub = rospy.Subscriber('waterLevel', Int16, waterCb)

    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        curLight += lightDesire - lightSen
        lightPub.publish(curLight)

        if time = 0 and waterLevel < waterDesire - waterRate/200.0: # level in 1 min
            waterPub.publish(True)
        else:
            waterPub.publish(False)


        rate.sleep()
