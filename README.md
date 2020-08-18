# Autonomous Agents TerraBot #

- [Overview](#overview)
  - [Software Architecture](#terrabot-software-architecture)
- [ROS Communication](#ros-communication)
- [Understanding the System](#understanding-the-system)
  - [TerraBot Node](#terrabot-node)
    + [Command line arguments](#command-line-arguments)
    + [Run time commands](#run-time-commands)
  - [Arduino Node](#arduino-node)
  - [Agent Node](#agent-node)
    + [Interactive Agent](#interactive-agent)
- [Simulator](#simulator)
  - [Installation](#installation)
  - [Running the Simulator](#running-the-simulator)
  - [Graphics](#graphics)
  - [Baseline File](#baseline-file)
  - [Speeding up Time](#speeding-up-time)
- [Additional Items](#additional-items)
  - [Health Ping](#health-ping)
  - [Frequency](#frequency)
  - [Camera](#camera)
  - [Interference](#interference-file)
  - [Time Series](#time-series)
- [Testing](#testing)
  - [Test File](#test-file)
    + [START](#start)
    + [QUIT](#quit)
    + [WAIT](#wait)
    + [ENSURE](#ensure)
  
    

## Overview ##
Welcome to the Autonomous Agents TerraBot project! For this project you and your partners
will be given a greenhouse outfitted with multiple sensors and actuators and a grow mat
with recently germinated seeds. The goal of the assignment is to provide the best environment
for the plants during a two week grow cycle. Each cycle will come with new challenges, so
be prepared!  

Each greenhouse contains two light, moisture, temperature, and humidity sensors, one water-level sensor, one current sensor, and one camera. For the redundant sensors, data values are contained in arrays, where index 0 contains the sensor reading of the first sensor and index 1 contains the sensor reading of the second sensor.  For the current sensor, the first index is the current, the second is the energy usage, to date.

| Name (topic name)            | Description                                                 | Message Type      | Range      |
| ---------------------------- | ----------------------------------------------------------- | ----------------- | ---------- |
| **_Sensors_**                |**_Use these to determine the system's state_**              |  **_—_**          |  **_—_**   |
| Current (cur)                | The current draw (index 0) and total energy usage (index 1) | Float32Array      |            |
| Light (light)                | Light intesnity in the system                               | Int32Array        | 0-600      |
| Water level (level)          | Height of the water in the reservoir (in mm)                | Float32           | 3300=15cm  |
| Temperature (temp)           | Internal temperature of the system                          | Int32Array        | in Celcius |
| Soil Moisture (smoist)       | The moisture of the pad  (higher is drier)                  | Int32Array        | 280-600    |
| Humidity (humid)             | Internal relative humidity  (%)                             | Int32Array        |  0-100     |
| Camera                       | Captures a photograph of stystem's state                    |   —               |   —        |
| **_Actuators_**              |**_Use these to adjust the system's state_**                 | **_—_**           |  **_—_**   |
| LED (led)                    | Adjust the power of the system's LED light fixture          | Int32             | 0-255      |
| Water Pump (wpump)           | Toggle whether the water pump is on or off                  | Bool              | True-False |
| Fan (fan)                    | Toggle whether the fan is on or off                         | Bool              | True-False |
| Frequency (freq)             | Adjust the sensing frequency of a specific sensor           | See lib/frqmsg.py |  —         |

Note: The water pump pumps 1cm depth of water (~.93cups) per minute.

### TerraBot Software Architecture ###
An Arduino communicates directly with the sensors and actuators, converts the raw data into clean data, and then forwards that data to a Raspberry Pi.
The Raspberry Pi, running ROS (Robot Operating System), receives the sensor data and makes it available
for your AI agent in the formats above. Additionally, it receives your agent's actuator commands as defined above
and relays them back to the Arduino. 

For development purposes, there is a simulator that does everything that the Arduino does on the real hardware.  The simulator can display graphics of the terrarium, showing the changes as actuators are turned on and off and plants grow.  Just as with the real hardware, you need to maintain a good environment for the simulated plants, or they will not flourish, and even die.

![System Diagram](https://github.com/reidgs/TerraBot/blob/master/system_diagram.jpg)


The above image shows an overview of the connections between the different nodes (ovals) in the system.
Pay particular attention to the topics (rectangles) connected to the agent, as those are the ones you will be using to
regulate your greenhouse.

## ROS Communication ##
In order to get your code working with the ROS messaging system,
follow the tutorial on the ROS website [here](https://wiki.ros.org/ROS/Tutorials).
Please look over the tutorials concerning ROS communication (Nodes, Topics, Publishers + Subscribers) to gain a general understanding of ROS.
You may find the other tutorials there helpful as well.


The TerraBot consists of three ROS *nodes* or processes, one for the Arduino (or simulator) communication with the sensors/actuators,
one relay node for publishing the clean data and listening for actuation commands,
and one that you will write for your agent.
The relay node *publishes* sensor data and *subscribes* to actuation commands over ROS topics
of specific types (shown above).
Your agent will be a ROS *node* which subscribes to each of the sensors topics, plans actions, and publishes to
actuators.
 
Be sure to check
your code to make sure that it is publishing and subscribing as you intend when bug fixing.

## Understanding the System ##
Running the TerraBot will start up a total of four nodes. Besides the three TerraBot nodes described above, it will also start up the roscore node, which regulates communications between the TerraBot nodes.

### TerraBot Node ###
The TerraBot node transfers actuator data from your agent to the Arduino/simulator node:
It subscribes to the topics to which your agent publishes and publishes to the topics to which the Arduino/simulator subscribes.

Vice versa, the TerraBot node transfers sensor data from the Arduino/simulator node to your agent:
It subscribes to the topics to which the Arudino/simulator publishes and publishes to the topics to which your agent subscribes.
 
In the transfering process, the data received by the TerraBot node are passed though functions via an external interference file. In order to reliably simulate errors which may happen by chance if run in the real world, the interference file
may be malicious and cause the sensor data and actuator commands to be corrupted in various ways before being sent on to the agent node.

#### Command Line Arguments ####
The following command line arguments are avaiable when running TerraBot.py:
    -h (--help): show help message and exit
    -v (--verbose): print more messages describing the workings of the system
    -l (--log): log all message traffic (in an auto-generated subdirectory of Log)
    -m (--mode) serial | sim : mode to run in (default is serial)
    -g (--graphics): Show graphical representation of simulation (only in sim mode)
    -s (--speedup) <value>: Increase simulated time (only in sim mode; automatically decreases speedup when fans or pump are on)
    -b (--baseline) <text file>: Initial clock time, sensor, and actuator values (only in sim mode)
    -i (--interference) <text file>: Set of instructions for when to manipulate sensor and actuator values
    -t (--test) <text file>: Tests correct execution based on a set of constraints that describe expected behavior
    -a (--agent) <python file>: Your agent program (if "none" - the default  - the agent node must be run externally)

#### Run Time Commands ####
Currently, the only run-time command is "q", which gracefully quits the system.  

WARNING: If you ^C out, sometimes not all the processes are killed.  You would then need to kill them (the ros, arduino/simulator, and agent processes) individually.  Especially if extra ROS nodes are running, unexpected interactions may occur. To simplify this, just run ./cleanup_processes from the TerraBot directory.

### Arduino Node ###
Sensors and actuators are being controlled in the Arduino node:

* The Arduino reads in all sensor data and translates them from raw values to more meaningful values that are then published to the TerraBot node. 
* The Arduino also subscribes to topics containing data values that are published by the TerraBot node. These meaningful values are translated to its raw form, with which the Arduino can write to the actuators.

All communication to and from the Arduino is done via the TerraBot node, meaning you should
never access the same topics as the Arduino. When not using the actual hardware, use the [simulator](#Simulator).

### Agent Node ###
The agent node is how you autonomously control the greenhouse. You will be able to access sensor data by subscribing to the topics to which the TerraBot publishes. You will also be able to write data to the actuators by publishing to the topics to which the TerraBot subscribes.

#### Interactive Agent ####
The interactive agent (agents/interactive_agent.py) enables you to interact with the TerraBot (either the real hardware or the simulator) using the same topics that your agent will be using. You start the interactive agent in a separate window from TerraBot.  The command line options are -s (--sim), to indicate that TerraBot is using the simulator and -l (--log), which prints the sensor data (this latter flag is not really very useful anymore, but is there for historical reasons).  

Once the interactive agent is started and connects to the TerraBot (while the two programs can be started in any order, it is usually better to start TerraBot first), you can send commands to the TerraBot via text input. The available commands are:
  q: quit
  f on / f off: turn fans on/off
  p on / p off: turn pump on/off
  l on / l off: turn leds on/off
  l <int>: turn leds to the given value, between 0 and 255
  c <filename>: take an image and store in the given file
  r <sensor> <val>: set the frequency of publishing the named sensor's values, in terms of seconds (e.g. 10 is 10 Hz, 0.1 is once every ten seconds, default is 1, for all sensors)
  s <int>: set the simulated time speedup to the given value
  v: print the current sensor values

*Note that the interactive agent can be run concurrently with other agents, so that you can view sensor values or turn on/off actuators manually while your autonomous system is running (this should be used only during development, not during testing).*

## Simulator ##

Since access to the actual greenhouse is somewhat limited, and things can take a long time, we have provided a simulator so that you can test your code before deploying it on the actual hardware. 

The simulator uses the same ROS topics as, and works in a way almost identical to, the Arduino node. The code for your agent and the TerraBot node is the exact same
as it would be on the Raspberry Pi. However, instead of having an Arduino node, the simulator comes with a
farduino (fake Arduino) node that mimics the actions of the Arduino node. This difference should
in no way affect how your code operates and should not be (significantly) noticeable from the perspective
of your agent node.

We are working to try to get the ranges of the sensors, the nominal values, and the rate of change of the sensors, to match the real world.  The values are approximate, however, so you should not assume that the real world and the simulator will behave exactly the same.

### Installation ###
The simulator and ROS require Ubuntu distributions. **We suggest installing a VirtualBox VM** on your computer so that you
can implement your agent. **We will provide you with a virtual machine that already has Ubuntu, ROS, and the TerraBot code installed.**

- VirtualBox: [Instructions here](https://www.wikihow.com/Install-VirtualBox) for installing
VirtualBox.

Our distribution contains:
* Ubuntu: download ubuntu 32 bit [here](https://ubuntu-mate.org/download/) to most
accurately mimic the software on the pi. (any ubuntu distro should be valid though this is the only one we have tested)
* ROS: [Instructions here](https://wiki.ros.org/melodic/Installation/Ubuntu_) for installing ROS.
Keep in mind that only the Desktop install is neccesary. Depending on your internet connection
this step may take a while.
* Panda3d, for graphics rendering
* TerraBot and simulator (lib/farduino.py) nodes

### Running the Simulator ###
In order to run the simulator, run the TerraBot with the simulator mode flag (-m sim), and optionally 1) if you wish to see a graphical representation of the terrarium (-g); 2) the multiplier you wish for the speed (-s <speedup>); and 3) a file that contains baseline (starting) values for the sensors, actuators, and the time which you would like the simulator to start at (seconds since midnight, day 1 of the simulated run) (-b <baselinefile>).  

>`./TerraBot.py -m sim -g -b param/default_baseline.bsl`  

### Graphics ###
The -g flag will enable display of a graphical representation of the simulation. The graphics window contains a 3D model of the terrarium and the plants within, along with a text panel containing information about the current environment, e.g., whether the pump is on or off, humidity, etc. Sounds are played to represent the pump and fan. The arrow keys and WASD can be used to navigate the scene, and the viewport can be reset by pressing 'r'. The camera will take pictures directly from this scene, from the perspective of the blue camera model, as in the real terrarium. Note that the camera <i>can still be used</i> even when the -g flag is not included, and the images produced will be equivalent to those taken with the graphics on.

### Baseline File ###
The simulator starts up with default values for the sensors, actuators, and clock.  You can create a 'baseline' .bsl text file to specify different initial values. To see the format in action, look in param/default_baseline.bsl. To specify a value, add a line in the format "name = value", optionally appending a comment (any characters after a "#" are ignored).

The possible values you can change are : 
> start, wpump, fan, led, temperature, humidity, smoist, wlevel, tankwater, plant_health, leaf_droop, lankiness

The "start" value is used to specify a starting time for the simulation. The plants will be grown automatically up to this date, according to the "leaf_droop", "lankiness", and "plant_health" values (see below). The format is either an int (the time in seconds) OR in the form DAY-HH:MM:SS (note that start=0 and start=1-00:00:00 are equivalent).

The "plant_health" value (a float in [0, 1]) will set the health of the plants from 0% healthy (0) to 100% healthy (1).

The "leaf_droop" value (a float in [0, 1]) will cause the leaves of the plants to droop. This normally happens when there is not enough water for the plant, but you can set it manually here.

The "lankiness" value (a float in [0, 1]) determines how lanky the plants are. A lanky plant is usually indicative of a lack of sunlight and vice versa, but you can set it manually here.

The rest are pretty self-explanatory. If a value not specified, the value in param/default_baseline.bsl is what will be used. All numbers can be floats, though "led" will be cast to an int.

### Speeding up Time ###
Note that much of what happens in a greenhouse happens very slowly, thus a speedup of 100 or more is recommended for development and testing.  However, what happens when actuators are on can happen very quickly (e.g., watering takes just a few seconds).  To accommodate this, the simulator automatically sets the speed low when either the pump or fans are on and then sets the speed to the user-desired value whenever they are both are off.  Your agent can also publish a "speedup" message to change the default speedup during run time, but this is not standard practice (and has no effect when operating on the actual hardware).

For example to run the simulator at 100x speed (i.e., 100 seconds of simulated time for every second of wall clock time), use:  
>`./TerraBot.py -m sim -s 100`  

*Note: to ensure consistency between your code in simulation and with the actual hardware, you should refrain from referring to outside functions (OS time.time()) and should instead refer to the ROS time topic via rospy.get_time().*

## Additional Items ##
There are some additional items that you need to know aobut in order to complete the assignments.  Specifically, they involve the health of your agent, the frequency of sensor readings, and access to the camera.

### Health Ping ###
Because of the long lasting nature of this project, it is possible that there may be unforeseen
errors in your code which will cause it to crash. Crashed code means no control over the system and
certain doom for your plants! In order to avoid this outcome, we have included restart functionality.  
The TerraBot will subscribe to the "ping" message (the data, a Boolean, is ignored).
If the TerraBot runs your agent (i.e., the -a flag is provided) and has not received a ping within
a set amount of time (default 6 minutes, either real or simulated time), it will assume your program has crashed and restart it automatically.

After a set number of crashes (currently 5), the TerraBot will quit.  There is a command-line option to send email when this happens, so that the team and the instructors can be notified.  Don't worry about how to do this - it is not necessary for development and testing, and we will use this feature only during the grow cycles.

### Frequency ###
It is our intention to eventually have you manage how much energy and water you use.  
Since reading the sensors takes energy, there is a way to tell the system how frequently for the Arduino will read from the sensors.
The more often the sensors are read, the more accurate your data will be, but the more energy you will use as well.
The 'freq' topic is used to indicate how often the sensors will be read. Use the "tomsg" function in freqmsg.py to convert a sensor name/frequency to a string that can be handled by ROS (see agents/interactive_agent.py for how exactly to do this).

For example, a frequency of 10 polls the given sensors at 10 Hz; a frequency of 0.1 polls the sensors once every 10 seconds.
Notice that this setting is variable, meaning it can be changed over the course of the deployment by sending a new 'freq' message.
This is useful, for instance, if you want to read the sensors infrequently until some event occurrs and then change the frequency to do better closed-loop control.  The default is 1 for all sensors.

### Camera ###
The camera is different from the rest of the sensors, as it is controlled directly by the Pi, and not by the Arduino. You can take a picture using the 'camera' topic; the single argument is the name of the file to store the JPEG image.  Note that, if you are using relative path name, the path is relative to the directory where you ran TerraBot, not to the directory you ran your agent.  **Make sure if the file path includes a directory that the directory actually exists, otherwise the image will not be saved.**

### Interference File ###
The interference file contains a schedule of times to manipulate the data being transferred between nodes. There are six functions, through one of which your data will be passed:

* normal : trasfers data directly without any modifications
* noise : slightly modifies data before transfering
* off : sets all data to 0 (or type equivelent)  
**_Only for sensors:_**
* low : sets a low value for that sensor 
* opt : sets an optimal value for that sensor 
* high : sets a high value for that sensor

*If no file is passed in, there will be no intereference in the transfer of data.*

### Time Series ###


## Testing ##
Your programming assignments will be graded automatically.  Trace files (.trc) will indicate what behaviors are expected to occur, and the grader, running in conjunction with the simulator, will check to see that all the conditions are successfully met.  The trace files to use can be specified on the command line using either the -t (one trace file) or -T (directory containing multiple trace files), along with the mode set to "grade" (e.g., -m grade).

You may test your agent by creating your own trace files (and your own interference files). We may give you example trace files, as well, but the actual grading will use trace files that you have not previously seen.

### Test File ###
The grader traces through commands given in this file and acts accordingly. The first line in the trace file is the address to the baseline file, and the second line is the address to the interference file. The commands for grading start on the third line of the trace file. The four commands available are: START, ENSURE, WAIT, and QUIT.

##### START #####
The START command starts the grading process. It does not take any arguments.

##### QUIT #####
The QUIT command terminates the current grading process. It does not take any arguments. If there are additional trace files that have not been traced through, the grader will start tracing through the next file, otherwise, the TerraBot will terminate all processes and exit. 

##### WAIT #####
The WAIT command will wait a certain amount of time for a value to evaluate to true. It takes in two arguments seperated by commas: the first argument is the expression being checked, and the second argument is the maximum wait time allowed. This command is finished and the next command starts once either: the first argument evaluates to true within the time frame (passed task), or the maximum wait time passes (failed task).
> EX: wait a maximum of 5 seconds for the led's value to be 255  
> `WAIT,grader_vars['led']==255,5` 

##### ENSURE #####
The ENSURE command will ensure that the value of the first argument given evaluates to true throughout the whole time period set. It takes in two arguments seperated by commas: the first argument is the expresion being checked, while the second argument is the length of the time period. This command is finished and the next command starts once either: the first argument evaluates to false (failed task), or the time period set passes (passed task).
> EX: ensure the water pump is on for at least 10 seconds  
>`ENSURE,grader_vars['wpump'],10` 
