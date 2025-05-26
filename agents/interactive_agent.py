#!/usr/bin/env python

import rclpy, rclpy.node
import sys, select, os
from std_msgs.msg import Float32, Int32, Int32MultiArray, Float32MultiArray, Bool, String
import argparse
import limits
from datetime import datetime
sys.path.insert(0, os.getcwd()[:os.getcwd().find('TerraBot')]+'TerraBot/lib')
from terrabot_utils import clock_time, get_ros_time, set_use_sim_time
from freqmsg import tomsg
from topic_def import sensor_types, actuator_types

rclpy.init()

class Sensors:
    time = 0
    light_level = 0
    moisture = 0
    humidity = 0
    temperature = 0
    weight = 0
    water_level = 0
    current = 0
    energy = 0
    light_level_raw = [0,0]
    moisture_raw = [0,0]
    humidity_raw = [0,0]
    temperature_raw = [0,0]
    weight_raw = [0, 0]


def moisture_reaction(data, sensors):
    sensors.moisture = (data.data[0] + data.data[1])/2.0
    sensors.moisture_raw = data.data
    if is_logging: print("    Moisture: %d %d" %(data.data[0], data.data[1]))

def humid_reaction(data, sensors):
    sensors.humidity = (data.data[0] + data.data[1])/2.0
    sensors.humidity_raw = data.data
    if is_logging: print("    Humidity: %d %d" %(data.data[0], data.data[1]))

def weight_reaction(data, sensors):
    # Each weight sensor holds half the weight of the pan
    sensors.weight = (data.data[0] + data.data[1])
    sensors.weight_raw = data.data
    if is_logging: print("    Weight: %d %d" %(data.data[0], data.data[1]))

def temp_reaction(data, sensors):
    sensors.temperature = (data.data[0] + data.data[1])/2.0
    sensors.temperature_raw = data.data
    if is_logging: print("    Temperature: %d %d" %(data.data[0], data.data[1]))

def light_reaction(data, sensors):
    sensors.light_level = (data.data[0] + data.data[1])/2.0
    sensors.light_level_raw = data.data
    if is_logging: print("    Lights: %d %d" %(data.data[0], data.data[1]))

def level_reaction(data, sensors):
    sensors.water_level = data.data
    if is_logging: print("    Level: %.2f" %data.data)

def cam_reaction(data):
    print ("picture taken\t" + data.data)

class Agent(rclpy.node.Node):
    def __init__(self, use_simulator):
        super().__init__("Interactive_Agent")
        set_use_sim_time(self, use_simulator)
        self.sensors = Sensors()
        self.update_time()

    def update_time (self):
        self.sensors.time = get_ros_time(self)
            
    def init_ros (self):
        self.pubs = {}
        for name in tdef.actuator_names:
            self.pubs[name] = self.create_publisher(
                                      tdef.actuator_types[name], name, 1)
        self.pubs['speedup'] = self.create_publisher(
                                      Int32, "speedup", 1)
            
        cbs = {'smoist': moisture_reaction, 'light': light_reaction,
               'level': level_reaction, 'temp': temp_reaction, 
               'humid': humid_reaction, 'weight': weight_reaction,}
        for name in tdef.sensor_names:
            self.subscription(sensor_types[name], name+"_output", cbs[name], 1)

    def publish (self, name, value):
        self.pubs[name].publish(value)

    def print_sensor_values (self):
        print("Sensor values at %s" % clock_time(sensors.time))
        print("  Light level: %.1f (%.1f, %.1f)"
              %(self.sensors.light_level, self.sensors.light_level_raw[0],
                self.sensors.light_level_raw[1]))
        print("  Temperature: %.1f (%.1f, %.1f)"
              %(self.sensors.temperature, self.sensors.temperature_raw[0],
                self.sensors.temperature_raw[1]))
        print("  Humidity: %.1f (%.1f, %.1f)"
              %(self.sensors.humidity, self.sensors.humidity_raw[0],
                self.sensors.humidity_raw[1]))
        print("  Soil moisture: %.1f (%.1f, %.1f)"
              %(self.sensors.moisture, self.sensors.moisture_raw[0],
                self.sensors.moisture_raw[1]))
        print("  Weight: %.1f (%.1f, %.1f)"
              %(self.sensors.weight, self.sensors.weight_raw[0],
                self.sensors.weight_raw[1]))
        print("  Reservoir level: %.1f" %self.sensors.water_level)

parser = argparse.ArgumentParser(description = "Interactive Agent")
parser.add_argument('-l', '--log', action = 'store_true',
                    help="print sensor values")
parser.add_argument('-s', '--sim', action = 'store_true', help="use simulator")
args = parser.parse_args()

is_logging = args.log

agent = Agent(args.sim)

# Wait for clock to start up correctly
while get_ros_time(agent) == 0:
    rclpy.spin_once(agent, timeout_sec=0.1)

# Give a chance for the initial sensor values to be read
while agent.sensors.temperature == 0:
    rclpy.spin_once(agent, timeout_sec=0.1)

print("Connected and ready for interaction")

while rclpy.ok():
    rclpy.spin_once(agent, timeout_sec=0.1)
    agent.update_time()

    ### Check for input
    if sys.stdin in select.select([sys.stdin],[],[],0)[0]:
        input = sys.stdin.readline()
        if input[0] == 'q':
            quit()
        else:
            try:
                if input[0] == 'f':
                    print("Turning fans %s" %("on" if (input.find("on") > 0) else "off"))
                    agent.publish('fan', input.find("on") > 0)
                elif input[0] == 'p':
                    print("Turning pump %s" %("on" if (input.find("on") > 0) else "off"))
                    agent.publish('wpump', input.find("on") > 0)
                elif input[0] == 'l':
                    level = (0 if (input.find("off") > 0) else
                             255 if (input.find("on") > 0) else int(input[1:]))
                    print("Adjusting light level to %d" %level)
                    agent.publish('led', level)
                elif input[0] == 'c':
                    print("Taking a picture, storing in %s" %input[2:-1])
                    agent.publish('camera', input[2:-1])
                elif input[0] == 'r':
                    sensor, freq = input[2:-1].split(" ")
                    msg = tomsg(sensor, float(freq))
                    if msg is not None:
                        print("Updating %s to frequency %s (every %.1f seconds)"
                              %(sensor, freq, 1/float(freq)))
                        agent.publish('freq', msg)
                elif input[0] == 'e':
                    sensor, period = input[2:-1].split(" ")
                    msg = tomsg(sensor, 1/float(period))
                    if msg is not None:
                        print("Updating %s to period of %s seconds (frequency of %f)"
                              %(sensor, period, 1/float(period)))
                        agent.publish('freq', msg)
                elif input[0] == 's':
                    agent.publish('speedup', int(input[1:]))
                elif input[0] == 'v':
                    agent.print_sensor_values()
                else:
                    print("Usage: q (quit)\n\tf [on|off] (fan on/off)\n\tp [on|off] (pump on/off)\n\tl [<level>|on|off] (led set to level ('on'=255; 'off'=0)\n\tr [smoist|cur|light|level|temp|humid][weight] [<frequency>] (update sensor to frequency)\n\te [smoist|cur|light|level|temp|humid][weight] [<period>] (update sensor to every <period> seconds)\n\tc <file> (take a picture, store in 'file')\n\ts [<speedup>] (change current speedup)\n\tv (print sensor values)")
            except:
                print("An error occurred and the action could not be executed")

    rclpy.sleep(1)
