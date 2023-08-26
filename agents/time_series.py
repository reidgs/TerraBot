#!/usr/bin/env python

import rospy, sys, select, os, time, argparse
from std_msgs.msg import Float32, Int32, Int32MultiArray, Float32MultiArray, Bool, String
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import limits
from log_data import write_log_data_line, process_log_data_line

class Subplots:
    name = None
    ax = None
    fig = None
    color = 'r'
    def __init__(self, the_name, the_axis, the_color, the_width,
                 the_force_update):
        self.name = the_name
        self.current = -100
        self.x = []
        self.y = []
        self.ax = the_axis
        self.color = the_color
        self.plot_width = the_width
        self.last_update = 0 # Time of last plotted value
        self.current = (0 if the_force_update else None)
        self.frequency = the_width/150 # in hours

    def update(self, time): # in hours since start
        if (self.current == None): return False
        if ((time - self.last_update) >= self.frequency):
            self.update1(time)
            return True
        elif (len(self.y) == 0):
            self.update1(time)
            return True
        else:
            # Plot if more than 10% change in value, regardless of time
            last_value = self.y[len(self.y)-1]
            if (abs(self.current - last_value) > 0.1*max(self.current,last_value)):
                self.update1(time)
                return True
        return False

    def update1(self, time): # time in hours since start
        min = max(0,float(time-self.plot_width))
        self.ax.set_xlim(left=min, right=min+(1.05*self.plot_width))
        self.x.append(time)
        self.y.append(self.current)
        self.ax.plot(self.x, self.y, self.color)
        self.last_update = time

subplotsG = {}
nrowsG = 0
ncolsG = 0

def init_ros (use_simulator):
    global led_pub, wpump_pub, fan_pub, ping_pub, camera_pub, subplotsG

    if use_simulator: rospy.set_param("use_sim_time", True)
    rospy.init_node("time_series_grapher", anonymous = True)

    rospy.Subscriber("smoist_output", Int32MultiArray,
                     update_sensor_multi_data, subplotsG['Soil Moisture'])
    rospy.Subscriber("light_output", Int32MultiArray,
                     update_sensor_multi_data, subplotsG['Light Level'])
    rospy.Subscriber("level_output", Float32,
                     update_sensor_data, subplotsG['Water Level'])
    rospy.Subscriber("temp_output", Int32MultiArray,
                     update_sensor_multi_data, subplotsG['Temperature'])
    rospy.Subscriber("humid_output", Int32MultiArray,
                     update_sensor_multi_data, subplotsG['Humidity'])
    rospy.Subscriber("weight_output", Float32MultiArray, 
                     update_sensor_multi_data, subplotsG['Weight'])
    rospy.Subscriber("led_input", Int32,
                     update_actuator_data, subplotsG['LEDs'])
    rospy.Subscriber("fan_input", Bool, update_actuator_data, subplotsG['Fan'])
    rospy.Subscriber("wpump_input", Bool,
                     update_actuator_data, subplotsG['Pump'])

def update_sensor_multi_data(data, subplot):
    subplot.current = (data.data[0] + data.data[1])/2
    write_log_data_line(log_file, subplot.name, data.data)

def update_sensor_data(data, subplot):
    subplot.current = data.data
    write_log_data_line(log_file, subplot.name, data.data)

def update_actuator_data(data, subplot):
    subplot.current = data.data
    write_log_data_line(log_file, subplot.name, data.data)

def add_time_series(fig, name, limits, force_update, color, pos, plot_width):
    global subplotsG, nrowsG, ncolsG
    ax = fig.add_subplot(nrowsG, ncolsG, pos)
    plt.title(name)
    subplotsG[name] = Subplots(name, ax, color, plot_width, force_update)
    ax.set_xlim(0, plot_width) # hours
    ax.yaxis.set_major_formatter(FormatStrFormatter('%d'))
    offset = limits[1]*0.1 # extend limits slightly
    ax.set_ylim(limits[0]-offset, limits[1]+offset)
    if (limits[0] == 0 and limits[1] == 1): # binary
        ax.set_yticks([0, 1]) #ax.set_yticks(['off','on'])

def init_plotting(plots, plot_width):
    global nrowsG, ncolsG
    fig = plt.figure()
    nrowsG, ncolsG = (5, 2)
    plt.subplots_adjust(hspace=0.7)
    plt.subplots_adjust(wspace=0.2)
    plt.ion()
    plt.rc('font', size=6)
    plt.rc('axes', titlesize=8)

    for plot in plots:
        add_time_series(fig, plot[0], plot[1], plot[2], plot[3], plot[4],
                        plot_width)

    return fig

def print_sensor_values():
    print("Light Level: %.2f" %subplotsG['Light Level'].current)
    print("Humidity: %.2f" %subplotsG['Humidity'].current)
    print("Temperature: %.2f" %subplotsG['Temperature'].current)
    print("Soil Moisture: %.2f" %subplotsG['Soil Moisture'].current)
    print("Weight: %.2f" %subplotsG['Weight'].current)
    print("Water Level: %.2f" %subplotsG['Water Level'].current)
    print("LEDs: %d" %subplotsG['LEDs'].current)
    print("Fan: %s" %("on" if subplotsG['Fan'].current else "off"))
    print("Pump: %s" %("on" if subplotsG['Pump'].current  else "off"))

parser = argparse.ArgumentParser(description = "Time Series Plotter")
parser.add_argument('-w', '--width', default = 24,
                    help="width of the plot, in hours")
parser.add_argument('-l', '--log', help="log the sensor data to file")
parser.add_argument('-r', '--replay', help="replay the sensor data from file")
parser.add_argument('-s', '--sim', action = 'store_true', help="use simulator")
parser.add_argument('-p', '--speedup', default = 1, help = 'replay playback speedup')
args = parser.parse_args()
plot_widthG = float(args.width) # in hours

replay_file = (None if not args.replay else open(args.replay, 'r', 0))
if (args.log):
    if (replay_file): print("WARNING: Cannot log while replaying")
    else: log_file = open(args.log, 'w+', 0)
else: log_file = None

plotsG = [('Light Level', limits.scale['light_level'], False, 'g', 1),
          ('Humidity', limits.scale['humidity'], False, 'g', 3),
          ('Temperature', limits.scale['temperature'], False, 'g', 5),
          ('Soil Moisture', limits.scale['moisture'], False, 'g', 7),
          ('Weight', limits.scale['weight'], False, 'g', 9),
          ('Water Level', limits.scale['water_level'], False, 'g', 10),
          ('LEDs', [0, 255], True, 'b', 2),
          ('Fan', [0, 1], True, 'b', 4),
          ('Pump', [0, 1], True, 'b', 6)]
fig = init_plotting(plotsG, plot_widthG)

def update_plots (hours_since_start):
    global plotsG, subplotsG, fig

    updated = False
    for plot in plotsG:
        if (subplotsG[plot[0]].update(hours_since_start)): updated = True
    if (updated):
        fig.canvas.draw()
        plt.pause(0.001)
        plt.show()
    return updated

def handle_stdin ():
    if sys.stdin in select.select([sys.stdin],[],[],0)[0]:
        input = sys.stdin.readline()
        if input[0] == 'q':
            plt.close()
            quit()
        elif input[0] == 'v':
            print_sensor_values()
        else:
            print("Usage:  q (quit)\n\tv (sensor values)")

if (replay_file):
    start_time = None
    for line in replay_file:
        cur_time, name, data = process_log_data_line(line)
        if (isinstance(data, tuple)): data = (data[0] + data[1])/2
        subplotsG[name].current = data
        if (not start_time): start_time = cur_time
        hours_since_start = (cur_time - start_time)/3600.0
        if (update_plots(hours_since_start)): time.sleep(1 / float(args.speedup))
        handle_stdin()

    print("Done replaying")
    time.sleep(5)
    replay_file.close()

else:
    init_ros(args.sim)
    rospy.sleep(2) # Do this even if running simulator to handle messages
    start_time = rospy.get_time()
    last_update = start_time

    # Update graph every so often, depending on plot width
    while not rospy.core.is_shutdown():
        hours_since_start = (rospy.get_time() - start_time)/3600.0
        update_plots(hours_since_start)
        handle_stdin()
        rospy.sleep(0.1)
