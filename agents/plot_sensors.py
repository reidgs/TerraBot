import rclpy, rclpy.node
import sys, select, time, argparse
from std_msgs.msg import Float32, Int32, Int32MultiArray, Float32MultiArray, Bool, String
import matplotlib.pyplot as plt
from limits import scale, limits, names
from topic_def import sensor_types, actuator_types
from terrabot_utils import set_use_sim_time, spin_for

class SensorPlots:
    def __init__(self):
        self.sensor_levels = {}
        self.actuator_labels  = {}
        self.sensor_bars = {}
        self.sensor_values = {}
        self.init_plotting()

    def add_sensor(self, name, fig, nrow, ncol, pos):
        self.sensor_levels[name] = fig.add_subplot(nrow, ncol, pos)
        self.update_sensor(name, None)

    def add_actuator(self, name, fig, nrow, ncol, pos):
        self.actuator_labels[name] = fig.add_subplot(nrow, ncol, pos)
        self.update_actuator(name, "")

    def update_sensor(self, name, value):
        ax = self.sensor_levels[name]
        if (value == None):
            value = 0
            self.sensor_values[name] = value
            ax.set_xlim(scale[name])
            bars = ax.barh(names[name], value)
            self.sensor_bars[name] = bars[0]
        self.sensor_bars[name].set_width(value)
        self.sensor_bars[name].set_color('Red' if value < limits[name][0] else
                                'Red' if value > limits[name][1] else 'Green')

    def update_actuator(self, name, actuator):
        ax = self.actuator_labels[name]
        if (not ax.texts):
            self.sensor_values[name] = ''
            ax.set_yticklabels([])
            #ax.set_yticks([],[])
            #ax.set_xticks([],[''])
            ax.bar(names[name],[0])
        else: 
            ax.texts[0].remove()
        ax.text(0, 0, actuator, ha='center', va='center', color='Blue')
    
    def update(self):
        for key in self.sensor_levels:
            self.update_sensor(key, self.sensor_values[key])
        for key in self.actuator_labels:
            self.update_actuator(key, self.sensor_values[key])
        plt.pause(0.0001)

    def init_plotting(self):
        fig = plt.figure()
        plt.subplots_adjust(hspace=0.6)
        plt.subplots_adjust(wspace=0.3)
        plt.ion()

        self.add_sensor('light_level', fig, 6,2,1)
        self.add_sensor('humidity', fig, 6,2,3)
        self.add_sensor('temperature', fig, 6,2,5)
        self.add_sensor('moisture', fig, 6,2,7)
        self.add_sensor('weight', fig, 6,2,9)
        self.add_sensor('water_level', fig, 6,2,10)

        self.add_actuator('led', fig, 6,2,2)
        self.add_actuator('fan', fig, 6,2,4)
        self.add_actuator('pump', fig, 6,2,6)

        plt.show()

    def print_sensor_values():
        print("Light Level: %.2f" %self.sensor_values['light_level'])
        print("Temperature: %.2f" %self.sensor_values['temperature'])
        print("Humidity: %.2f" %self.sensor_values['humidity'])
        print("Weight: %.2f" %self.sensor_values['weight'])
        print("Soil Moisture: %.2f" %self.sensor_values['moisture'])
        print("Water Level: %.2f" %self.sensor_values['water_level'])
        print("LED: %s" %self.sensor_values['led'])
        print("Fan: %s" %self.sensor_values['fan'])
        print("Water Pump: %s" %self.sensor_values['pump'])

class Plotter(rclpy.node.Node):
    def __init__(self, use_simulator):
        super().__init__("plotter")
        set_use_sim_time(self, use_simulator)
        self.sensorPlots = SensorPlots()
        self.init_ros()

    def init_ros (self):
        self.create_subscription(sensor_types['smoist'], "smoist_output",
                     lambda x: self.update_sensor_multi_data(x, 'moisture'), 1)
        self.create_subscription(sensor_types['light'], 'light_output',
                     lambda x: self.update_sensor_multi_data(x, 'light_level'), 1)
        self.create_subscription(sensor_types['level'], "level_output",
                     lambda x: self.update_sensor_data(x, 'water_level'), 1)
        self.create_subscription(sensor_types['temp'], "temp_output",
                     lambda x: self.update_sensor_multi_data(x, 'temperature'), 1)
        self.create_subscription(sensor_types['humid'], "humid_output",
                     lambda x: self.update_sensor_multi_data(x, 'humidity'), 1)
        self.create_subscription(sensor_types['weight'], "weight_output",
                     lambda x: self.update_sensor_multi_data(x, 'weight'), 1)
        self.create_subscription(actuator_types['led'], "led_input", 
                     lambda x: self.update_sensor_data(x, 'led'), 1)
        self.create_subscription(actuator_types['fan'], "fan_input", 
                     lambda x: self.update_binary_data(x, 'fan'), 1)
        self.create_subscription(actuator_types['wpump'], "wpump_input", 
                     lambda x: self.update_binary_data(x, 'pump'), 1)

        # Initialize actuators
        self.sensorPlots.sensor_values['led'] = 'Off'
        self.sensorPlots.sensor_values['fan'] = 'Off'
        self.sensorPlots.sensor_values['pump'] = 'Off'

    def update_sensor_multi_data(self, data, name):
        self.sensorPlots.sensor_values[name] =(data.data[0] + data.data[1])/2

    def update_sensor_data(self, data, name):
        self.sensorPlots.sensor_values[name] = data.data

    def update_binary_data(self, data, name):
        self.sensorPlots.sensor_values[name] = ('On' if data.data else 'Off')


parser = argparse.ArgumentParser(description = "Interactive Agent")
parser.add_argument('-s', '--sim', action = 'store_true', help="use simulator")
args = parser.parse_args()

rclpy.init()
plotter = Plotter(args.sim)
time.sleep(2) # Need to do this even if running simulator to handle messages

while rclpy.ok():
    plotter.sensorPlots.update()

    if sys.stdin in select.select([sys.stdin],[],[],0)[0]:
        input = sys.stdin.readline()
        if input[0] == 'q':
            quit()
        elif input[0] == 'v':
            print_sensor_values()
        else:
            print("Usage:  q (quit)\n\tv (sensor values)")

    spin_for(plotter, 1)
