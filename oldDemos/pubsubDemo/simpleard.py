#!/usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import Int16, Bool
import csv
import override as ovr

bag = rosbag.Bag('newTest.bag', 'w')
sensorsR = {'time':['photoCell', 'waterLevel']}
actuatorsR = {'time':['LEDs', 'pump']}
sensorsF = {'time':['photoCell', 'waterLevel']}
actuatorsF = {'time':['LEDs', 'pump']}
size = 0

rospy.init_node('Pi', anonymous=True)
lightPubF = rospy.Publisher('lightSensorF', Int16, queue_size=10)
waterPubF = rospy.Publisher('waterLevelF', Bool, queue_size=10)
lightSub = rospy.Subscriber('lightSensorR', Int16, lightRCb)
waterSub = rospy.Subscriber('waterLevelR', Int16, wtrCb)
rate = rospy.Rate(3)

lightPubR = rospy.Publisher('LEDsR', Int16, queue_size=10)
lightSubF = rospy.Subscriber('LEDsF', Int16, lightFCb)

def time():
    return rospy.get_rostime().secs

def LEDCb(data):
    lightR = lightF(data.data)
    lightPubR.publish(lightR)
    time = time()
    actuatorsF.update({time: [ data.data , actuatorsF.get(time,[None,None])[1] ]})
    actuatorsR.update({time: [ lightR, actuatorsR.get(time,[None,None])[1] ]})



def lightRCb(data):
    lightF = lightR(data.data)
    lightPubF.publish(lightF)
    #global bag
    #bag.write('/lightHaveAmt', data)
    time = time()
    raw.update({time: [ data.data , raw.get(time,[None,None])[1] ]})
    filt.update({time: [ lightF, filt.get(time,[None,None])[1] ]})

def wtrCb(data):
    waterF = waterR(data.data)
    waterPubF.publish(waterF)
    #global bag
    #bag.write('/waterLevelAmt', data)
    time = rospy.get_rostime().secs
    raw.update({time: [ raw.get(time,[None,None])[0] , data.data ]})
    filt.update({time: [ filt.get(time,[None,None])[0] , waterF ]})

def writeCSV():
    global raw
    global filt
    with open('filt.csv', 'a') as f:
        for key in sorted(filt.keys()):
            f.write("%s,%s,%s\n"%(key,filt[key][0],filt[key][1]))

    with open('raw.csv', 'a') as f: 
        for key in sorted(raw.keys()):
            f.write("%s,%s,%s\n"%(key,raw[key][0],raw[key][1]))
    raw = {'time':['light','water']}
    filt = {'time':['light','water']}


#while not rospy.is_shutdown():
if len(d) == 10:
    writeCSV()
   # rate.sleep

if rospy.is_shutdown():
    for topic, msg, t in \
            bag.read_messages(topics=['/lightHaveAmt','/waterLevelAmt']):
        time = "time: %d.%d" %(t.secs, t.nsecs)
        print(topic,msg,time)
    bag.close()
rospy.spin()
