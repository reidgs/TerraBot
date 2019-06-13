#!/usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import Int16, Bool
import csv

lightSen=0
curlight=0

def lightCb(data):
    global lightSen
    lightSen = data.data
    rospy.loginfo("light sensor: %s", data.data)



if __name__ == '__main__':
    rospy.init_node('student', anonymous=True)
    lightPub = rospy.Publisher('lightGive', Int16, queue_size=10)
    lightSub = rospy.Subscriber('lightHave', Int16, lightCb)
    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        lightPub.publish(500-lightSen+curlight)
        curlight = 500-lightSen+curlight
        rate.sleep()







