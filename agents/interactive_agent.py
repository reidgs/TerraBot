#!/usr/bin/env python

import rospy, sys, select
from std_msgs.msg import Float32, Int32, Int32MultiArray, Float32MultiArray, Bool, String
import argparse
import plot, limits

class Sensors:
    time = 0
    light_level = 0
    moisture = 0
    humidity = 0
    temperature = 0
    water_level = 0
    current = 0
    energy = 0

parser = argparse.ArgumentParser(description = "Interactive Agent")
parser.add_argument('-l', '--log', action = 'store_true',
                    help="print sensor values")
parser.add_argument('-s', '--sim', action = 'store_true', help="use simulator")
parser.add_argument('-p', '--plot', action = 'store_true',
                    help="plot sensor values")
args = parser.parse_args()

sensorsG = Sensors()
is_logging = args.log
is_plotting = args.plot
use_simulator = args.sim

def init_sensors():
    global sensorsG
    sensorsG.time = rospy.get_time()

### ROS-related stuff
### Set up publishers, subscribers, and message handlers

def init_ros ():
    global led_pub, wpump_pub, fan_pub, ping_pub, sensorsG

    if use_simulator: rospy.set_param("use_sim_time", True)
    rospy.init_node("interactive_agent", anonymous = True)

    led_pub = rospy.Publisher("led_input", Int32, latch = True, queue_size = 1)
    wpump_pub = rospy.Publisher("wpump_input", Bool, latch = True,
                                queue_size = 1)
    fan_pub = rospy.Publisher("fan_input", Bool, latch = True, queue_size = 1)

    ping_pub = rospy.Publisher("ping", Bool, latch = True, queue_size = 1)
#    ping_pub = rospy.Publisher("ping", Int32, latch = True, queue_size = 1)

    rospy.Subscriber("smoist_output", Int32MultiArray,
                     moisture_reaction, sensorsG)
    rospy.Subscriber("light_output", Int32MultiArray, light_reaction, sensorsG)
    rospy.Subscriber("level_output", Float32, level_reaction, sensorsG)
    rospy.Subscriber("temp_output", Int32MultiArray, temp_reaction, sensorsG)
    rospy.Subscriber("humid_output", Int32MultiArray, humid_reaction, sensorsG)
    rospy.Subscriber("cur_output", Float32MultiArray, power_reaction, sensorsG)

def moisture_reaction(data, sensorsG):
    sensorsG.moisture = (data.data[0] + data.data[1])/2
    if is_logging: print("    Moisture: %d %d" %(data.data[0], data.data[1]))

def humid_reaction(data, sensorsG):
#    sensorsG.humidity = (data.data[0] + data.data[1])/2
    sensorsG.humidity = data.data[0]
    if is_logging: print("    Humidity: %d %d" %(data.data[0], data.data[1]))

def temp_reaction(data, sensorsG):
#    sensorsG.temperature = (data.data[0] + data.data[1])/2
    sensorsG.temperature = data.data[0]
    if is_logging: print("    Temperature: %d %d" %(data.data[0], data.data[1]))

def light_reaction(data, sensorsG):
#    sensorsG.light_level = (data.data[0] + data.data[1])/2
    sensorsG.light_level = data.data[0]
    if is_logging: print("    Lights: %d %d" %(data.data[0], data.data[1]))

def level_reaction(data, sensorsG):
    sensorsG.water_level = data.data
    if is_logging: print("    Level: %.2f" %data.data)

def power_reaction(data, sensorsG):
    sensorsG.current = -data.data[0]
    sensorsG.energy = -data.data[1]
    if is_logging:
        print("    Current: %f Energy: %f" %(sensorsG.current, sensorsG.energy))

def cam_reaction(data):
    print ("picture taken\t" + data.data)

def ping():
    global last_ping
    print("PING! %.1f" %sensorsG.time)
    last_ping = sensorsG.time
    ping_pub.publish(True)
#    ping_pub.publish(last_ping)

if is_plotting: plot.init_plotting()

init_ros()
init_sensors()
rospy.sleep(2) # Give a chance for the initial sensor values to be read
ping()

while not rospy.core.is_shutdown():
    sensorsG.time = rospy.get_time()

    # Ping every 3 minutes, twice as frequently as timeout in TerraBot
    if (sensorsG.time - last_ping) >= 18000: ping()

    ### Check for input
    if sys.stdin in select.select([sys.stdin],[],[],0)[0]:
        input = sys.stdin.readline()
        if input[0] == 'q':
            quit()
        elif input[0] == 'f':
            print("Turning fans %s" %("on" if (input.find("on") > 0) else "off"))
            fan_pub.publish(input.find("on") > 0)
        elif input[0] == 'p':
            print("Turning pump %s" %("on" if (input.find("on") > 0) else "off"))
            wpump_pub.publish(input.find("on") > 0)
        elif input[0] == 'l':
            level = (0 if input.find("off") else
                     255 if input.find("on") else int(input[1:]))
            print("Adjust light level to %d" %level)
            led_pub.publish(level)
        else:
            print("Usage: q (quit)\n\tf [on|off] (fan on/off)\n\tp [on|off] (pump on/off)\n\tl [<level>|on|off] (led set to level ('on'=255; 'off'=0)")
    if is_plotting:
        plot.update_sensor('moisture', sensorsG.moisture)
        plot.update_sensor('humidity', sensorsG.humidity)
        plot.update_sensor('temperature', sensorsG.temperature)
        plot.update_sensor('light_level', sensorsG.light_level)
        plot.update_sensor('water_level', sensorsG.water_level)
        plot.update_sensor('current', sensorsG.current)
        # Plot average power (energy/time)
#        plot.update_sensor('energy', sensorsG.energy/sensorsG.time)
        plot.update_sensor('energy', sensorsG.energy)

        plot.plt.pause(0.0001)

    rospy.sleep(1)
