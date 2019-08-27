import matplotlib.pyplot as plt
from limits import scale, limits, names

sensor_levels = {}
state_labels  = {}
sensor_bars = {}

def add_sensor(name, fig, nrow, ncol, pos):
    global sensor_levels
    sensor_levels[name] = fig.add_subplot(nrow, ncol, pos)
    update_sensor(name, None)

def add_state(name, fig, nrow, ncol, pos):
    global state_levels
    state_labels[name] = fig.add_subplot(nrow, ncol, pos)
    update_state(name, "")

def update_sensor(name, value):
    global sensor_levels, names, sensor_bars
    ax = sensor_levels[name]
    if (value == None):
        if (value == None): value = 0
        ax.set_xlim(scale[name])
        bars = ax.barh(names[name], value)
        sensor_bars[name] = bars[0]
    sensor_bars[name].set_width(value)
    sensor_bars[name].set_color('Red' if value < limits[name][0] else
                                'Green' if value >= limits[name][1] else 'Blue')

def update_state(name, state):
    global state_labels, names
    ax = state_labels[name]
    if (not ax.texts):
       ax.set_yticklabels([])
       ax.set_yticks([],[])
       ax.set_xticks([],[''])
       ax.bar(names[name],[0])
    else: 
        ax.texts[0].remove()
    ax.text(0, 0, state, ha='center', va='center', color='Blue')

def init_plotting():
    fig = plt.figure()
    plt.subplots_adjust(hspace=0.6)
    plt.subplots_adjust(wspace=0.3)
    plt.ion()

    add_sensor('light_level', fig, 6,2,1)
    add_state('lighting_fsm', fig, 6,2,2)

    add_sensor('water_level', fig, 6,1,2)
    add_sensor('moisture', fig, 6,2,5)
    add_state('watering_fsm', fig, 6,2,6)

    add_sensor('humidity', fig, 6,2,7)
    add_state('humidity_fsm', fig, 6,2,8)

    add_sensor('temperature', fig, 6,2,9)
    add_state('temperature_fsm', fig, 6,2,10)

    add_sensor('current', fig, 6,2,11)
    add_sensor('energy', fig, 6,2,12)

    plt.show()

#init_plotting()
#update_sensor('light_level', 100)
#update_state('lighting_fsm', 'lighting')
#plt.pause(5)
