import rospy, sys, select, os, time, argparse
from std_msgs.msg import Float32, Int32, Int32MultiArray, Float32MultiArray, Bool, String
import matplotlib.pyplot as plt
from limits import scale, limits, names

sensor_levels = {}
actuator_labels  = {}
sensor_bars = {}
sensor_values = {}

def add_sensor(name, fig, nrow, ncol, pos):
    global sensor_levels
    sensor_levels[name] = fig.add_subplot(nrow, ncol, pos)
    update_sensor(name, None)

def add_actuator(name, fig, nrow, ncol, pos):
    global actuator_labels
    actuator_labels[name] = fig.add_subplot(nrow, ncol, pos)
    update_actuator(name, "")

def update_sensor(name, value):
    global sensor_levels, names, sensor_bars, sensor_values
    ax = sensor_levels[name]
    if (value == None):
        if (value == None): value = 0
        sensor_values[name] = value
        ax.set_xlim(scale[name])
        bars = ax.barh(names[name], value)
        sensor_bars[name] = bars[0]
    sensor_bars[name].set_width(value)
    sensor_bars[name].set_color('Red' if value < limits[name][0] else
                                'Red' if value > limits[name][1] else 'Green')

def update_actuator(name, actuator):
    global actuator_labels, names, sensor_values
    ax = actuator_labels[name]
    if (not ax.texts):
        sensor_values[name] = ''
        ax.set_yticklabels([])
        #ax.set_yticks([],[])
        #ax.set_xticks([],[''])
        ax.bar(names[name],[0])
    else: 
        ax.texts[0].remove()
    ax.text(0, 0, actuator, ha='center', va='center', color='Blue')

def init_ros (use_simulator):
    global led_pub, wpump_pub, fan_pub, ping_pub, camera_pub
    global sensor_values, actuator_labels

    if use_simulator: rospy.set_param("use_sim_time", True)
    rospy.init_node("time_series_grapher", anonymous = True)

    rospy.Subscriber("smoist_output", Int32MultiArray,
                     update_sensor_multi_data, 'moisture')
    rospy.Subscriber("light_output", Int32MultiArray,
                     update_sensor_multi_data, 'light_level')
    rospy.Subscriber("level_output", Float32,
                     update_sensor_data, 'water_level')
    rospy.Subscriber("temp_output", Int32MultiArray,
                     update_sensor_multi_data, 'temperature')
    rospy.Subscriber("humid_output", Int32MultiArray,
                     update_sensor_multi_data, 'humidity')
    rospy.Subscriber("weight_output", Float32MultiArray,
                     update_sensor_multi_data, 'weight')
    rospy.Subscriber("cur_output", Float32MultiArray, update_power_data, 'cur')
    rospy.Subscriber("led_input", Int32, update_sensor_data, 'led')
    rospy.Subscriber("fan_input", Bool, update_binary_data, 'fan')
    rospy.Subscriber("wpump_input", Bool, update_binary_data, 'pump')

    # Initialize actuators
    sensor_values['led'] = 'Off'
    sensor_values['fan'] = 'Off'
    sensor_values['pump'] = 'Off'

def update_sensor_multi_data(data, name):
    sensor_values[name] =(data.data[0] + data.data[1])/2

def update_sensor_data(data, name):
    sensor_values[name] = data.data

def update_binary_data(data, name):
    sensor_values[name] = ('On' if data.data else 'Off')

def update_power_data(data, name):
    sensor_values['current'] = data.data[0]
    sensor_values['energy'] = data.data[1]

def init_plotting():
    fig = plt.figure()
    plt.subplots_adjust(hspace=0.6)
    plt.subplots_adjust(wspace=0.3)
    plt.ion()

    add_sensor('light_level', fig, 6,2,1)
    add_sensor('humidity', fig, 6,2,3)
    add_sensor('temperature', fig, 6,2,5)
    add_sensor('moisture', fig, 6,2,7)
    add_sensor('weight', fig, 6,2,9)
    add_sensor('water_level', fig, 6,2,10)
    add_sensor('current', fig, 6,2,11)
    add_sensor('energy', fig, 6,2,12)

    add_actuator('led', fig, 6,2,2)
    add_actuator('fan', fig, 6,2,4)
    add_actuator('pump', fig, 6,2,6)

    plt.show()

def print_sensor_values():
    print("Light Level: %.2f" %sensor_values['light_level'])
    print("Temperature: %.2f" %sensor_values['temperature'])
    print("Humidity: %.2f" %sensor_values['humidity'])
    print("Weight: %.2f" %sensor_values['weight'])
    print("Soil Moisture: %.2f" %sensor_values['moisture'])
    print("Water Level: %.2f" %sensor_values['water_level'])
    print("LED: %s" %sensor_values['led'])
    print("Fan: %s" %sensor_values['fan'])
    print("Water Pump: %s" %sensor_values['pump'])

parser = argparse.ArgumentParser(description = "Interactive Agent")
parser.add_argument('-s', '--sim', action = 'store_true', help="use simulator")
args = parser.parse_args()

init_plotting()

init_ros(args.sim)
rospy.sleep(2) # Need to do this even if running simulator to handle messages

while not rospy.core.is_shutdown():
    for key in sensor_levels:
        update_sensor(key, sensor_values[key])
    for key in actuator_labels:
        update_actuator(key, sensor_values[key])
    plt.pause(0.0001)

    if sys.stdin in select.select([sys.stdin],[],[],0)[0]:
        input = sys.stdin.readline()
        if input[0] == 'q':
            quit()
        elif input[0] == 'v':
            print_sensor_values()
        else:
            print("Usage:  q (quit)\n\tv (sensor values)")

    rospy.sleep(1)
