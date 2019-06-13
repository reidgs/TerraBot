#!/usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import Int16, Bool
import csv

lightGiven = 0

def lightCb(data):
    global lightGiven
    lightGiven = data.data

if __name__ == '__main__':
    rospy.init_node('Simulator', anonymous=True)
    lightPub = rospy.Publisher('lightHave', Int16, queue_size=10)
    lightSub = rospy.Subscriber('lightGive', Int16, lightCb)
    rate = rospy.Rate(10)
    wait = rospy.Rate(3)
    while not rospy.is_shutdown():
        with open('testSim.csv') as csvfile:
            readCSV = csv.reader(csvfile, delimiter=',')
            for row in readCSV:
                rospy.loginfo("csv: %s lightStdGive: %s " % (row[1], lightGiven))
                lightPub.publish(int(row[1])+lightGiven)
                wait.sleep()
        rate.sleep()



