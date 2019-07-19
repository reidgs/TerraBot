#!/usr/bin/env python
import subprocess
import rospy
from std_msgs.msg import Bool
import time
import os

last_pinged = 


def ping_cb(data):
    last_pinged = time.time()

rospy.init_node('run', anonymous = True)
ping_sub = rospy.Subscriber("ping", Bool, ping_cb)


prog = subprocess.Popen('python relay.py -l & python student.py')

while __name__ == "__main__":
    if time.time() - last_pinged > 60:





