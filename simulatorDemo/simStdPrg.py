#!/usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import Int32, Float32, Bool

simulator = True
timeNow = 0
timeFreq = 1
timeRange = 10

lightSen = -500
curLight = 0
lightDesire = 500
lowerLight = 0
upperLight = 600

waterLevel = 50
pumpRate = 73 #cubicIn/min
waterDesire = 50 #mm
pumpTime = 0 #min
pumpOn = False

lightMsg = False

def timeCb(data):
    global timeNow
    timeNow = data.data


#def getTime():
 #   if simulator:
  #      return timeNow
   # else:
    #    return rospy.get_rostime().secs


def lightCb(data):
    global lightSen
    global lightMsg
    lightSen = data.data
    rospy.loginfo("light sensor: %s", data.data)
    lightMsg = True

def waterCb(data):
    global waterMsg
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

    #simulated time
    simTime = rospy.Subscriber('simTime', Int32, timeCb)

    while not rospy.is_shutdown():
        #checks if new message so not redundant
        if lightSen != -500 and lightMsg:
            curLight += lightDesire - lightSen
            if curLight > upperLight:
                curLight = upperLight
            elif curLight < lowerLight:
                curLight = lowerLight
            lightPub.publish(curLight)
            lightMsg = False
        #pump not already on & need to turn on
        if timeNow > pumpTime and waterLevel <= waterDesire - 5: #pumpRate/200.0: # level in 1 min
            pumpTime = (waterDesire-waterLevel)*3/5 + getTime()
            #pumpTime = (waterDesire-waterLevel)/(pumpRate/200.0)*60 + getTime()
            waterPub.publish(True)
            pumpOn = True
        #pump needs to get turned off
        elif timeNow > pumpTime and pumpOn:
            pumpOn = False
            waterPub.publish(False)


