#!/usr/bin/env python

import rclpy, rclpy.node
import sys, select, os, time, argparse
from std_msgs.msg import Float32, Int32, Int32MultiArray, Float32MultiArray, Bool, String
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import limits
from log_data import write_log_data_line, process_log_data_line
from terrabot_utils import get_ros_time, set_use_sim_time

plotsG = [('Light Level', limits.scale['light_level'], False, 'g', 1),
          ('Humidity', limits.scale['humidity'], False, 'g', 3),
          ('Temperature', limits.scale['temperature'], False, 'g', 5),
          ('Soil Moisture', limits.scale['moisture'], False, 'g', 7),
          ('Weight', limits.scale['weight'], False, 'g', 9),
          ('Water Level', limits.scale['water_level'], False, 'g', 10),
          ('LEDs', [0, 255], True, 'b', 2),
          ('Fan', [0, 1], True, 'b', 4),
          ('Pump', [0, 1], True, 'b', 6)]

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

class TimeSeries(rclpy.node.Node):
    def __init__(self, use_simulator):
        super().__init__("Time_Series_Grapher")
        set_use_sim_time(self, use_simulator)
        self.subplots = {}
        self.nrows = 0
        self.ncols = 0
        self.init_ros()

    def subscribe_sensor(self, msg_name, type, plot_name):
        self.create_subscription(type, msg_name,
                                 lambda data: self.update_sensor_data(
                                                          data, plot_name), 1)

    def subscribe_msensor(self, msg_name, type, plot_name):
        self.create_subscription(type, msg_name,
                                 lambda data: self.update_sensor_multi_data(
                                                          data, plot_name), 1)

    def subscribe_actuator(self, msg_name, type, plot_name):
        self.create_subscription(type, msg_name,
                                 lambda data: self.update_actuator_data(
                                                          data, plot_name), 1)
    def init_ros (self):
        self.subscribe_msensor("smoist_output", Int32MultiArray, 'Soil Moisture')
        self.subscribe_msensor("light_output", Int32MultiArray, 'Light Level')
        self.subscribe_sensor("level_output", Float32, 'Water Level')
        self.subscribe_msensor("temp_output", Int32MultiArray,'Temperature')
        self.subscribe_msensor("humid_output", Int32MultiArray,'Humidity')
        self.subscribe_msensor("weight_output", Float32MultiArray, 'Weight')
        self.subscribe_actuator("led_input", Int32, 'LEDs')
        self.subscribe_actuator("fan_input", Bool, 'Fan')
        self.subscribe_actuator("wpump_input", Bool, 'Pump')

    def update_sensor_multi_data(self, data, subplot_name):
        subplot = self.subplots[subplot_name]
        subplot.current = (data.data[0] + data.data[1])
        if (subplot_name != 'Weight'): subplot.current /= 2
        write_log_data_line(log_file, subplot_name, data.data, self)

    def update_sensor_data(self, data, subplot_name):
        self.subplots[subplot_name].current = data.data
        write_log_data_line(log_file, subplot_name, data.data, self)

    def update_actuator_data(self, data, subplot_name):
        self.subplots[subplot_name].current = data.data
        write_log_data_line(log_file, subplot_name, data.data, self)

    def add_time_series(self, fig, name, limits, force_update,
                        color, pos, plot_width):
        ax = fig.add_subplot(self.nrows, self.ncols, pos)
        plt.title(name)
        self.subplots[name] = Subplots(name, ax, color, plot_width, force_update)
        ax.set_xlim(0, plot_width) # hours
        ax.yaxis.set_major_formatter(FormatStrFormatter('%d'))
        offset = limits[1]*0.1 # extend limits slightly
        ax.set_ylim(limits[0]-offset, limits[1]+offset)
        if (limits[0] == 0 and limits[1] == 1): # binary
            ax.set_yticks([0, 1]) #ax.set_yticks(['off','on'])

    def init_plotting(self, plots, plot_width):
        self.fig = plt.figure()
        self.nrows, self.ncols = (5, 2)
        plt.subplots_adjust(hspace=0.7)
        plt.subplots_adjust(wspace=0.2)
        plt.ion()
        plt.rc('font', size=6)
        plt.rc('axes', titlesize=8)

        for plot in plots:
            self.add_time_series(self.fig, plot[0], plot[1], plot[2],
                                 plot[3], plot[4], plot_width)

    def print_sensor_values(self):
        print("Light Level: %.2f" %self.subplots['Light Level'].current)
        print("Humidity: %.2f" %self.subplots['Humidity'].current)
        print("Temperature: %.2f" %self.subplots['Temperature'].current)
        print("Soil Moisture: %.2f" %self.subplots['Soil Moisture'].current)
        print("Weight: %.2f" %self.subplots['Weight'].current)
        print("Water Level: %.2f" %self.subplots['Water Level'].current)
        print("LEDs: %d" %self.subplots['LEDs'].current)
        print("Fan: %s" %("on" if self.subplots['Fan'].current else "off"))
        print("Pump: %s" %("on" if self.subplots['Pump'].current  else "off"))

    def update_plots (self, hours_since_start):
        updated = False
        for plot in plotsG:
            if (self.subplots[plot[0]].update(hours_since_start)):
                updated = True
        if (updated):
            self.fig.canvas.draw()
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
            time_series.print_sensor_values()
        else:
            print("Usage:  q (quit)\n\tv (sensor values)")

parser = argparse.ArgumentParser(description = "Time Series Plotter")
parser.add_argument('-w', '--width', default = 24,
                    help="width of the plot, in hours")
parser.add_argument('-l', '--log', help="log the sensor data to file")
parser.add_argument('-r', '--replay', help="replay the sensor data from file")
parser.add_argument('-s', '--sim', action = 'store_true', help="use simulator")
parser.add_argument('-p', '--speedup', default = 1, help = 'replay playback speedup')
args = parser.parse_args()

replay_file = (None if not args.replay else open(args.replay, 'r', 0))
if (args.log):
    if (replay_file): print("WARNING: Cannot log while replaying")
    else: log_file = open(args.log, 'w+', 0)
else: log_file = None

rclpy.init()
time_series = TimeSeries(args.sim)

time_series.init_plotting(plotsG, float(args.width)) # plot width in hours

if (replay_file):
    start_time = None
    for line in replay_file:
        cur_time, name, data = process_log_data_line(line)
        if (isinstance(data, tuple)): data = (data[0] + data[1])/2
        self.subplots[name].current = data
        if (not start_time): start_time = cur_time
        hours_since_start = (cur_time - start_time)/3600.0
        if (time_series.update_plots(hours_since_start)):
            time.sleep(1 / float(args.speedup))
        handle_stdin()

    print("Done replaying")
    time.sleep(5)
    replay_file.close()

else:
    # Wait for clock to start up correctly
    while get_ros_time(time_series) == 0:
        rclpy.spin_once(time_series, timeout_sec=0.1)

    # Give a chance for the initial sensor values to be read
    while time_series.subplots['Temperature'].current == 0:
        print("XXX", time_series.subplots['Temperature'].last_update)
        rclpy.spin_once(time_series, timeout_sec=0.1)

    start_time = get_ros_time(time_series)
    last_update = start_time

    # Update graph every so often, depending on plot width
    while rclpy.ok():
        hours_since_start = (get_ros_time(time_series) - start_time)/3600.0
        time_series.update_plots(hours_since_start)
        handle_stdin()
        rclpy.spin_once(time_series, timeout_sec=0.1)
