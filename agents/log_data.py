#!/usr/bin/env python

import rclpy, rclpy.node, array
from std_msgs.msg import Float32, Int32, Int32MultiArray, Float32MultiArray, Bool, String
from terrabot_utils import get_ros_time, set_use_sim_time
from topic_def import sensor_names, sensor_types, actuator_names, actuator_types

class Logger(rclpy.node.Node):
    def __init__(self, use_simulator, log_file):
        super().__init__("logger")
        self.log_file = log_file
        set_use_sim_time(self, use_simulator)
        self.init_ros()

    def init_ros(self):
        for topic in sensor_names:
            self.create_subscription(sensor_types[topic], topic+"_output",
                                     lambda x: self.log_data_cb(x, topic), 1)
        for topic in actuator_names:
            self.create_subscription(actuator_types[topic], topic+"_input",
                                     lambda x: self.log_data_cb(x, topic), 1)

    def log_data_cb(self, data, topic):
        write_log_data_line(self.log_file, topic, data.data, get_ros_time(self))
        
def write_log_data_line(log_file, name, data, now):
    if (log_file):
        if (type(data) in [tuple, list, array.array]):
            log_file.write("%f '%s' %.1f %.1f\n"
                           %(now, name, data[0], data[1]))
        elif (isinstance(data, int)):
            log_file.write("%f '%s' %d\n" %(now, name, data))
        elif (isinstance(data, float)):
            log_file.write("%f '%s' %.1f\n" %(now, name, data))
        else:
            log_file.write("%f '%s' %s\n" %(now, name, data))

def process_log_data_line(line):
    sline = line.split("'")
    data = sline[2].strip(' \n').split(' ')
    return (float(sline[0]), sline[1],
            ((float(data[0]), float(data[1])) if (len(data) > 1) else
            float(data[0]) if ('.' in data[0]) else
            int(data[0]) if (data[0].isdigit()) else data[0]))

def read_log_file(filename):
    log_data = []
    with open(filename) as log_file:
        for line in log_file:
            log_data.append(process_log_data_line(line))
    return log_data

def log_sensordata(log_data, time, sensordata={}, next_index=0):
    for index in range(next_index, len(log_data)):
        datum = log_data[index]
        #print(index, datum)
        if (datum[0] > time): return (sensordata, index)
        else:
            sensordata[datum[1]] = datum[2]
    return (sensordata, len(log_data))

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description = "Logger")
    parser.add_argument('file', help="log the sensor data to file")
    parser.add_argument('-s', '--sim', action = 'store_true', help="use simulator")
    args = parser.parse_args()

    rclpy.init()
    with open(args.file, "w") as log_file:
        logger = Logger(args.sim, log_file)
        rclpy.spin(logger)
