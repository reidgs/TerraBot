#!/usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import Int16, Bool
import csv

bag = rosbag.Bag('newTest.bag', 'w')
d = {'time':['light','water']}
size = 0

lghtHave = 0
lghtNeed = 0
wtrLvl = 0

def lghtCb(data):
    global lghtHave
    lghtHave = data.data
    global bag
    bag.write('/lightHaveAmt', data)
    time = rospy.get_rostime().secs
    d.update({time: [ data.data , d.get(time,[None,None])[1] ]})

def wtrCb(data):
    global wtrLvl
    wtrLvl = data.data
    global bag
    bag.write('/waterLevelAmt', data)
    time = rospy.get_rostime().secs
    d.update({time: [ d.get(time,[None,None])[0] , data.data ]})


if __name__=='__main__':
    rospy.init_node('Pi', anonymous=True)
    lghtpub = rospy.Publisher('lightNeedAmt', Int16, queue_size=10)
    wtrpub = rospy.Publisher('pump', Bool, queue_size=10)
    lghtsub = rospy.Subscriber('lightHaveAmt', Int16, lghtCb)
    wtrsub = rospy.Subscriber('waterLevelAmt', Int16, wtrCb)
    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        if wtrLvl < 300:
            pumpswitch = True
        else:
            pumpswitch = False
        need = int(lghtHave*225/600)
        lghtpub.publish(need)
        wtrpub.publish(pumpswitch)
        rate.sleep()
        if len(d) == 10:
            print(d)
            with open('test.csv', 'a') as f:
                for key in sorted(d.keys()):
                    f.write("%s,%s,%s\n"%(key,d[key][0],d[key][1]))
            d={'time':['light','water']}

    if rospy.is_shutdown():
        for topic, msg, t in bag.read_messages(topics=['/lightHaveAmt','/waterLevelAmt']):
            time = "time: %d.%d" %(t.secs, t.nsecs)
            print(topic,msg,time)
        bag.close()

