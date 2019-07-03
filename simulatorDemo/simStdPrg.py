#!/usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import Float32, Bool
import csv

lightSen = 0
curLight = 0
lightDesire = 500

waterLevel = 50
pumpRate = 73 #cubicIn/min
waterDesire = 50 #mm
pumpTime = 0 #min

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
    lightPub = rospy.Publisher('lightGive', Float32, queue_size=10)
    lightSub = rospy.Subscriber('lightHave', Float32, lightCb)

    waterPub = rospy.Publisher('pumpOn', Bool, queue_size=10)
    waterSub = rospy.Subscriber('waterLevel', Float32, waterCb)

    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        curLight += lightDesire - lightSen
        lightPub.publish(curLight)

        if rospy.get_rostime().secs > pumpTime and waterLevel < waterDesire - 6: #pumpRate/200.0: # level in 1 min
            pumpTime = (waterDesire-waterLevel)/2 + rospy.get_rostime().secs
            #pumpTime = (waterDesire-waterLevel)/(pumpRate/200.0)*60 + rospy.get_rostime().secs
        if rospy.get_rostime().secs < pumpTime:
            waterPub.publish(True)
            rospy.loginfo("what")
        else:
            waterPub.publish(False)


        rate.sleep()
