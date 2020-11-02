#!/usr/bin/env python
import rospy

def write_log_data_line(log_file, name, data):
    if (log_file):
        if (isinstance(data, tuple)):
            log_file.write("%f '%s' %.1f %.1f\n"
                           %(rospy.get_time(), name, data[0], data[1]))
        elif (isinstance(data, int)):
            log_file.write("%f '%s' %d\n" %(rospy.get_time(), name, data))
        elif (isinstance(data, float)):
            log_file.write("%f '%s' %.1f\n" %(rospy.get_time(), name, data))
        else:
            log_file.write("%f '%s' %s\n" %(rospy.get_time(), name, data))

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
    import sys, select, os
    sys.path.insert(0, os.getcwd()[:os.getcwd().find('TerraBot')]+'TerraBot/lib')
    import topic_def as td
    from std_msgs.msg import Float32, Int32, Int32MultiArray, Float32MultiArray, Bool, String
    import argparse

    parser = argparse.ArgumentParser(description = "Interactive Agent")
    parser.add_argument('file', help="log the sensor data to file")
    parser.add_argument('-s', '--sim', action = 'store_true', help="use simulator")
    args = parser.parse_args()

    if args.sim: rospy.set_param("use_sim_time", True)
    rospy.init_node("logger", anonymous = True)

    def log_data_cb(data, file_and_topic):
        write_log_data_line(file_and_topic[0], file_and_topic[1], data.data)

    with open(args.file, "w") as log_file:
        for topic in td.sensor_names:
            rospy.Subscriber(topic+"_output", td.sensor_types[topic],
                             log_data_cb, (log_file, topic))
        for topic in td.actuator_names:
            rospy.Subscriber(topic+"_input", td.actuator_types[topic],
                             log_data_cb, (log_file, topic))
        while not rospy.core.is_shutdown():
            rospy.sleep(0.1)
