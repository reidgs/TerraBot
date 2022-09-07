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
    + [DELAY](#delay)
    + [STOP/QUIT](#stop-or-quit)
    + [WHENEVER](#whenever)
      + [WAIT](#wait)
      + [ENSURE](#ensure)
      + [SET](#set)
      + [PRINT](#print)

## Overview ##
Welcome to the Autonomous Agents TerraBot project! For this project you and your partners
will be given a greenhouse outfitted with multiple sensors and actuators and a rockwool "soil" planted with radish and lettuce
seeds. The goal of the assignment is to provide the best environment
for the plants during a two week grow cycle. Each cycle will come with new challenges, so be prepared!  

Each greenhouse contains two temperature, humidity, light, moisture, and weight sensors, one water-level sensor, one current sensor, and one camera. For the redundant sensors, data values are contained in arrays, where index 0 contains the sensor reading of the first sensor and index 1 contains the sensor reading of the second sensor.  For the current sensor, the first index is the current, the second is the energy usage, to date.  The camera is a bit different - you send a command to take a picture and the image is saved to a given file location. 

The greenhouse also has three actuators - fans, LEDs, and a water pump.  The LED's light level can be set between 0-255.  The fans and pump can be turned on (True) or off (False).  In addition, you can adjust the frequency at which a particular sensor reports data and send a "ping" message periodically to the TerraBot software, to let it know that your agent is still running.

| Name (topic name)            | Description                                                 | Message Type      | Range      |
| ---------------------------- | ----------------------------------------------------------- | ----------------- | ---------- |
| **_Sensors_**                |**_Use these to determine the system's state_**              |  **_—_**          |  **_—_**   | 
| Temperature (temp)           | Internal temperature of the greenhouse (in Celcius)         | Int32Array        | in Celcius | 
| Humidity (humid)             | Internal relative humidity  (%)                             | Int32Array        |  0-100     | 
| Light (light)                | Light intesnity in the greenhouse                           | Int32Array        | 0-1000     | 
| Soil Moisture (smoist)       | The moisture of the "soil"  (higher is wetter)              | Int32Array        | 280-600    | 
| Weight (weight)              | Weighs soil and plants (in grams); Total weight is average of the 2 sensors | Float32Array      |            |
| Water level (level)          | Height of the water in the reservoir (in mm)                | Float32           | 3300=15cm  | 
| Current (cur)                | The current draw (index 0) and total energy usage (index 1) | Float32Array      |            |
| Camera                       | Captures a photograph of stystem's state                    |   —               |   —        | 
| **_Actuators_**              |**_Use these to adjust the system's state_**                 | **_—_**           |  **_—_**   | 
| Fan (fan)                    | Toggle whether the fans are on or off                       | Bool              | True-False | 
| LED (led)                    | Adjust the power of the system's LED light fixture          | Int32             | 0-255      |
| Water Pump (wpump)           | Toggle whether the water pump is on or off                  | Bool              | True-False |
| **_Additional_**              |**_Use these to adjust the system's state_**                | **_—_**           |  **_—_**   | 
| Frequency (freq)             | Adjust the sensing frequency of a specific sensor           | See lib/frqmsg.py |  —         |
| Ping (ping)                  | Let the TerraBot know that your agent is still alive        | Bool              | True       |

Note: The water pump pumps approximately 1cm depth of water (~.93cups) per minute.

For optimal growing parameters, see TerraBot/agents/limits.py

### TerraBot Software Architecture ###
An Arduino communicates directly with the sensors and actuators, converts the raw data into clean data, and then forwards that data to a Raspberry Pi.
The Raspberry Pi, running ROS (Robot Operating System), receives the sensor data and makes it available
to your AI agent in the formats above. Additionally, it receives your agent's actuator commands, as defined above,
and relays them back to the Arduino. 

For development purposes, there is a simulator that does most everything that the Arduino does on the real hardware.  The simulator can display graphics of the terrarium, showing the changes as actuators are turned on and off and plants grow.  Just as with the real hardware, you need to maintain a good environment for the simulated plants, or they will not flourish, and even die.  **Note: What happens in the simulator is similar to the real greenhouse, but not exact.  Thus, you should be sure to thoroughly test all your code on the real hardware before deploying your agent to grow plants!**

The diagram below shows an overview of the connections between the different nodes (ovals) in the system.
Pay particular attention to the topics (rectangles) connected to the agent, as those are the ones you will be using to
regulate your greenhouse.

![System Diagram](https://github.com/reidgs/TerraBot/blob/master/system_diagram.jpg)

## ROS Communication ##
In order to get your code working with the ROS messaging system,
follow the tutorial on the ROS website [here](https://wiki.ros.org/ROS/Tutorials).
Please look over the tutorials concerning ROS communication (Nodes, Topics, Publishers + Subscribers) to gain a general understanding of ROS.
You may find the other tutorials there helpful as well.

The TerraBot consists of three ROS *nodes* or processes:
* the Arduino (real hardware) or farduino (simulator) node communicates with the sensors and actuators
* the TerraBot node publishes the data, optionally adding noise to it, listens for actuation commands, and optionally checks whether a given set of constraints hold on the execution of the system
* the Agent node (which you will write) controls the actuators based on sensor readings and other data

The Arduino and TerraBot nodes *publish* sensor data and *subscribe* to actuation commands over ROS topics
of specific types (shown above).
Your agent will be a ROS *node* which subscribes to each of the sensors topics, plans actions, and publishes to
actuators.
 
Be sure to check
your code to make sure that it is publishing and subscribing as you intend when bug fixing.

## Understanding the System ##
Running TerraBot.py will start up three processes. Besides starting the TerraBot and Arduino nodes described above, it will also start up the _roscore_ node, which regulates communications between all the other nodes.

### TerraBot Node ###
The TerraBot node transfers actuator data from your agent to the Arduino/farduino (simulator) node:
It subscribes to the topics to which your agent publishes and publishes to the topics to which the Arduino/simulator subscribes.

Vice versa, the TerraBot node transfers sensor data from the Arduino/simulator node to your agent:
It subscribes to the topics to which the Arudino/simulator publishes and publishes to the topics to which your agent subscribes.
 
In the transfering process, the data received by the TerraBot node are (optionally) passed though functions specified by an _interference file_. In order to reliably simulate errors which may happen by chance if run in the real world, an interference file may specify adding noise to the sensor data, or cause a sensor or actuator to act as if it were broken (including being stuck on).  A robust agent should be able to handle a variety of such issues, which often do manifest themselves on the real hardware.

#### Command Line Arguments ####
The following command line arguments are available when running TerraBot.py:
```
  -h (--help): show help message and exit
  -v (--verbose): print more messages describing the workings of the system
  -l (--log): log all message traffic (in an auto-generated subdirectory of Log)
  -m (--mode) serial | sim : mode to run in (default is serial on the real greenhouse, sim is simulator)
  -g (--graphics): Show graphical representation of simulation (only in sim mode)
  -s (--speedup) <value>: Increase simulated time (only in sim mode; automatically decreases speedup when fans or pump are on)
  -b (--baseline) <text file>: Initial clock time, sensor, and actuator values (only in sim mode)
  -i (--interference) <text file>: Set of instructions for when to manipulate sensor and actuator values
  -t (--test) <text file>: Tests correct execution based on a set of constraints that describe expected behavior
  -a (--agent) <python file>: Your agent program (if "none" - the default  - the agent node must be run externally)
  -f (--fixedshutter) <value>: Use the given value as the camera shutter speed, rather than adjusting dynamically based on light level
```
#### Run Time Commands ####
The following commands can be given while TerraBot.py is running:
```
  q: gracefully quit the system
  t: print out the current time (especially useful when running in simulated mode without the graphics displayed)
```

**WARNING: If you ^C out, sometimes not all the processes are killed.  You would then need to kill them (the roscore, arduino/simulator, and agent processes) individually.  Especially if extra ROS nodes are running, unexpected interactions may occur. To simplify this, just run _./cleanup_processes_ from the TerraBot directory.**

### Arduino Node ###
Sensors and actuators are being controlled in the Arduino node:

* The Arduino reads in all sensor data and translates them from raw values to more meaningful values that are then published to the TerraBot node. 
* The Arduino also subscribes to topics containing data values that are published by the TerraBot node. These meaningful values are translated to its raw form, with which the Arduino can write to the actuators.

All communication to and from the Arduino is done via the TerraBot node, meaning you should
never access the same topics as the Arduino. When not using the actual hardware, use the [simulator](#Simulator).

### Agent Node ###
The agent node is how you autonomously control the greenhouse. You will be able to access sensor data by subscribing to the topics to which the TerraBot publishes. You will also be able to write data to the actuators by publishing to the topics to which the TerraBot subscribes.  Your agent will base its decisions on the sensor data it receives, some saved state data, and a schedule of behaviors to run. 

#### Interactive Agent ####
The interactive agent (agents/interactive_agent.py) enables you to interact with the TerraBot (either the real hardware or the simulator) using the same topics that your agent will be using. This program can be very useful for exploring how the TerraBot works, debugging, and checking on the statues of sensors.  You start the interactive agent in a separate window from TerraBot.  The command line options are:
```
-s (--sim), to indicate that TerraBot is using the simulator
-l (--log), which prints the sensor data (this latter flag is not really very useful anymore, but is there for historical reasons)
```

Once the interactive agent is started and connects to the TerraBot (while the two programs can be started in any order, it is usually better to start TerraBot first), you can send commands to the TerraBot via text input. The available commands are:
```
  q: quit
  f on / f off: turn fans on/off
  p on / p off: turn pump on/off
  l on / l off: turn leds on/off
  l <int>: turn leds to the given value, between 0 and 255
  c <filename>: take an image and store in the given file
  r <sensor> <val>: set the frequency of publishing the named sensor's values, in terms of seconds (e.g. 10 is 10 Hz, 0.1 is once every ten seconds, default is 1, for all sensors)
  e <sensor> <val>: set the period of publishing the named sensor's values, in seconds (for convenience - period is 1/frequency)
  s <int>: set the speedup time to the given value (simulator only)
  v: print the current sensor values
```
*Note that the interactive agent can be run concurrently with other agents, so that you can view sensor values or turn on/off actuators manually while your autonomous system is running (this should be used only during development, not during testing).*

## Simulator ##

Since access to the actual greenhouse is somewhat limited, and growing plants can take a long time, we have provided a graphics-based simulator so that you can test your code before deploying it on the actual hardware. 

The simulator uses the same ROS topics as, and works in a way almost identical to, the Arduino node. The code for your agent and the TerraBot node is the exact same
as it would be on the Raspberry Pi. However, instead of having an Arduino node, the simulator comes with a
farduino (fake Arduino) node that mimics the actions of the Arduino node. This difference should
in no way affect how your code operates and should not be (significantly) noticeable from the perspective
of your agent node.

We are working to try to get the ranges of the sensors, the nominal values, and the rate of change of the sensors, to match the real world.  The values are approximate, however, so you should not assume that the real world and the simulator will behave exactly the same.

### Installation ###
The simulator and ROS require Ubuntu distributions. **We suggest installing a VirtualBox VM** on your computer so that you
can implement your agent. **We will provide you with a virtual machine that already has installed Ubuntu, ROS, the TerraBot code, and various python packages that you will need for your assignments.**

[Instructions are here](https://www.wikihow.com/Install-VirtualBox) for installing VirtualBox.
If you want to install your own version of the system on an Ubuntu machine, see the instructions in README.virtualbox (and ignore the specific instructions related to VirtualBox).

[Instructions are here](M1SETUP.md) for installing natively on M1 arm64 (Apple Silicon) devices.

### Running the Simulator ###
To run the simulator, start TerraBot.py with the simulator mode flag (-m sim), and optionally 1) -g, if you wish to see a graphical representation of the terrarium; 2) -s <speedup>, to specify the multiplier you wish for the speed; and 3) -b <baselinefile>, a file that contains baseline (starting) values for the sensors, actuators, and the time which you would like the simulator to start at (seconds since midnight, day 1 of the simulated run).  

>`./TerraBot.py -m sim -g -b param/default_baseline.bsl`  

### Graphics ###
We have included an option to display a graphical representation of the TerraBot hardware and the state of the simulation (see below).  The display also includes a representation of the growing plants, showing how they develop (and either thrive or die) over time.

<p align="center">
  <img src="https://github.com/reidgs/TerraBot/blob/master/simulator.JPG">
</p>

The graphics display is enabled using the -g flag when running TerraBot. The graphics window contains a 3D model of the terrarium and the plants within, along with a text panel containing information about the current environment, e.g., whether the pump is on or off, humidity, etc. Sounds are played to represent the pump and fan. The arrow keys and WASD can be used to navigate the scene, and the viewport can be reset by pressing 'r'. The camera will take pictures directly from this scene, from the perspective of the blue camera model, as in the real terrarium. Note that the camera <i>can still be used</i> even when the -g flag is not included, and the images produced will be equivalent to those taken with the graphics on.

### Baseline File ###
The simulator starts up with default values for the sensors, actuators, and clock.  You can create a 'baseline' (.bsl) text file to specify different initial values. This enables you to set up specific scenarios that you want to test for, rather than waiting for them to arise at some point in time during the simulation. 

The file param/default_baseline.bsl illustrates the format of the baseline files. In particular, to specify a value, add a line in the format "name = value", optionally appending a comment (any characters after a "#" are ignored).  The possible values you can change are : 
> start, wpump, fan, led, temperature, humidity, smoist, wlevel, tankwater, plant_health, leaf_droop, lankiness

The "start" value is used to specify a starting time for the simulation. The plants will be grown automatically up to this date, according to the "leaf_droop", "lankiness", and "plant_health" values (see below). The format is either an int (the time in seconds) OR in the form DAY-HH:MM:SS (note that start=0 and start=1-00:00:00 are equivalent).

The "plant_health" value (a float in [0, 1]) will set the health of the plants from 0% healthy (0) to 100% healthy (1).

The "leaf_droop" value (a float in [0, 1]) will cause the leaves of the plants to droop. This normally happens when there is not enough water for the plant, but you can set it manually here.

The "lankiness" value (a float in [0, 1]) determines how lanky the plants are. A lanky plant is usually indicative of a lack of sunlight and vice versa, but you can set it manually here.

The rest are pretty self-explanatory. If a value is not specified, the value in param/default_baseline.bsl is what will be used. All numbers can be floats, though "led" will be cast to an int.

### Speeding up Time ###
Note that much of what happens in a greenhouse happens very slowly, thus a speedup of 100 or more is recommended for development and testing.  However, what happens when actuators are on can happen very quickly (e.g., watering takes just a few seconds).  To accommodate this, the simulator automatically sets the speed low when either the pump or fans are on and then sets the speed to the user-desired value whenever they are both are off.  Your agent can also publish a "speedup" message to change the default speedup during run time, but this is not standard practice (and has no effect when operating on the actual hardware).

For example to run the simulator at 100x speed (i.e., 100 seconds of simulated time for every second of wall clock time), use:  
>`./TerraBot.py -m sim -s 100`  

*Note: to ensure consistency between your code in simulation and with the actual hardware, you should refrain from referring to outside functions (OS time.time()) and should instead refer to the ROS time topic via rospy.get_time().*

## Additional Items ##
There are some additional items that you may need to know about in order to complete the assignments.  Specifically, they involve the health of your agent, the frequency of sensor readings, and access to the camera.

### Camera ###
The camera is different from the rest of the sensors, as it is controlled directly by the Pi, and not by the Arduino. You can take a picture using the 'camera' topic; the single argument is the name of the file to store the JPEG image.  **Note that, if you are using relative path name, the path is relative to the directory where you ran TerraBot, not to the directory you ran your agent!**  **Make sure the file path includes a directory that actually exists, otherwise the image will not be saved!**
  
### Health Ping ###
Because of the long lasting nature of this project, it is possible that there may be unforeseen
errors in your code which will cause it to crash. Crashed code means no control over the system and
certain doom for your plants! In order to avoid this outcome, we have included restart functionality.  To help handle this, the TerraBot subscribes to the "ping" message (the data, a Boolean, is ignored).
  
If the TerraBot runs your agent (i.e., the -a flag is provided) and has not received a ping within
a set amount of time (default 6 minutes, either real or simulated time), it will assume your program has crashed and restart it automatically.
After a set number of crashes (currently 5), the TerraBot will quit.  There is a command-line option to send email when this happens, so that the team and the instructors can be notified.  Don't worry about how to do this - it is not necessary for development and testing, and we will use this feature only during the grow cycles, if you request it.

### Frequency ###
It is our intention to eventually have you manage how much energy and water you use.  Since reading the sensors takes energy, there is a way to tell the system how frequently for the Arduino will read from the sensors. The more often the sensors are read, the more accurate your data will be, but the more energy you will use as well.
The 'freq' topic is used to indicate how often the sensors will be read. Use the "tomsg" function in freqmsg.py to convert a sensor name/frequency to a string that can be handled by ROS (see agents/interactive_agent.py for how exactly to do this).

For example, a frequency of 10 polls the given sensors at 10 Hz; a frequency of 0.1 polls the sensors once every 10 seconds.
Notice that this setting is variable, meaning it can be changed over the course of the deployment by sending a new 'freq' message.
This is useful, for instance, if you want to read the sensors infrequently until some event occurrs (such as a behavior that uses that sensor is enabled) and then change the frequency to do better closed-loop control.  The default is 1 for all sensors.

### Interference File ###
Probably the one factor that makes reliable autonomous systems hard to develop is that unexpected events occur frequently.  While these can be rare and occur unpredictably, it is important for your agent to detect and handle anomalies that arise.  To that end, we have implemented a mechanism where it can seem as though sensors and actuators are noisy or not working correctly.  Which sensors and actuators are malfunctioning and what times, and how they are malfunctioning, are described in an *interference file*.  The interference file contains a schedule of times to manipulate the data being transferred between nodes. Interference files can be used in conjunction with the simulator or the real hardware, although it is mainly intended to be used with the simulator to simulate sensor and actuator failures.

For sensor data and the LEDs, the available modification constraints are:
```
  normal : transfers data directly without any modifications
  noise : slightly modifies data before transfering, adding Gaussian noise
  <value> : the sensor is stuck at that value
```
For the fan and water pump, the available modification constraints are:
```
  normal : transfers data directly without any modifications (the default)
  off : the actuator is stuck off  
  on : the actuator is stuck on
```
The fan, pump, LEDs, and water level (wlevel) all are singletons, and the modification constraints are indicated thusly:
```
wpump = off
```
The rest of the sensors are redundant, and one has to indicate the modification constraints for each, separately:
```
humidity = [normal, noise]
temperature = [noise, 20.0]
```
One can specify a sequence of modifications that take effect at different times, by placing an "AT" command before a set of modification constraints.
```
AT 1-03:00:00       # Starting at 3am, the first day
light = [normal, 0] # Right light sensor is stuck off
fan = off           # Fans are not working

AT 1-04:30:00       # Starting at 4:30am
light = [normal, normal] # Lights are now working
```
Note that 1) comments can be placed at the end of lines and 2) the time format is day-HH:MM:SS, where 1-00:00:00 is the beginning of the run.

*If no file is passed in, there will be no intereference in the transfer of data.*

### Time Series ###
It is often useful to visualize how the sensor and actuator data change over time.  For instance, to see how quickly environmental variables (such as moisture and humidity) change or to see whether the fans turn on at regular intervals.  To facilitate this, the time series program (agents/time_series.py) provides a way to visualize the sensor data and actuator commands over time.  The program connects to the TerraBot node and subscribes to all the topics that the TerraBot handles (except for frequency and speedup).  It can optionally log the data for later replay, so that you can analyze how things are working or input data to a machine learning algorithm. 

The program displays a set of windows, one for each topic, and plots the topic values over time (the X-axis).  A window scrolls when the values approach the right side of the window.  The Y-axis for the fans and pump are discrete (either 0/off or 1/on), while the Y-axis for the sensors and LEDs are continuous.  The sensor values are plotted in green and the actuator values in blue.

The command line options are:
```
    -h (--help): show help message and exit
    -s (--sim): use the simulator
    -w (--width) <value>: width of the windows, in hours (indicates how much data can be shown at once and how quickly it scrolls)
    -l (--log) <filename>: log the sensor data to the file
    -r (--replay) <filename>: replay the sensor data from the file
    -s (--speedup) <value>: playback speed for replaying (not to be confused with the speedup of the simulator)
```
In addition, there are two run-time commands:
```
  q: quit the program
  v: print the current sensor and actuator values
```

## Testing ##
Your programming assignments will be graded automatically.  Test files (.tst) will indicate what behaviors are expected to occur, and the tester will check to see that all the conditions are successfully met.  The test files to use can be specified on the command line using the -t option, followed by the .tst file. 

We strongly recommend creating your own baseline and test files to evaluate situations that you think might occur in real life - experience shows that unexpected combinations of factors often occur in practice, especially when running the system for two weeks growing real plants.  Creating a comprehensive set of test environments (and sharing these with others) is just good practice.

### Test File ###
Test files consist of several parts.  First, one can specify a baseline and/or interference file within a test file.  This is indicated as such:
```
BASELINE = smoist_up.bsl
INTERFERENCE = smoist_up.inf
```
If multiple baseline or interference lines are included, only the last one in the file is used.  Also, the specifications in the test file override any command line specifications.

#### DELAY ####
Next, one can specify how long to wait before applying any of the test constraints (see below).  It is useful to delay starting to test for a period of time to give the agent a chance to initialize (for instance, if it takes a while to produce an initial schedule).  This constraint can be specified in one of two ways:
```
DELAY FOR <value> # Wait "value" seconds before starting to test
DELAY UNTIL day-HH:MM:SS # Wait until the given time (measured from the start of the TerraBot operating)
```  
So, for instance, "DELAY UNTIL 1-03:30:00" would wait for 3.5 hours before starting to apply the test constraints.  As above, the last DELAY constraint in the file is used.

#### STOP or QUIT ####
Third, one can specify how long to test for.  The STOP constraint just ends testing, the QUIT constraint stops testing and causes the TerraBot program to quit.  For both variants, the time until ending can be specified either using seconds or date-time:
```
STOP AFTER 36000 # Stop testing after 10 hours
QUIT AT 3-23:59:59 # Run testing for 3 full days, and then quit the simulator
```
Note again that comments can be place at the end of lines

#### WHENEVER ####
The bulk of test files consist of WHENEVER constraints.  These are subtests that are activiated whenever a particular condition is met.  WHENEVER constraints consist of a trigger and a body, which is a sequence of WAIT, ENSURE, SET, and PRINT subconstraints that specify what behavior is expected of the system.

The triggers for WHENEVER constraints can be a Boolean relation or a date-time. The Boolean relations can consist of numbers, sensor values, and actuator states (**light, temperature, humidity, smoist, weight, current, wlevel, led, wpump, fan, camera, ping, time, mtime**).  **time** is the clock time, in seconds; **mtime** is the number of seconds past midnight -- you can get the hour of the day using (mtime//3600).  The date-time trigger is specified in terms of the first occurrence, and every time it finishes, another 24 hours are added on to the trigger time. Examples include:
```
WHENVER smoist[0] < 450 or smoist[1] < 450 # Every time either soil moisture sensor gets below 450
WHENEVER wpump # Every time the pump is turned on
WHENEVER 1-00:00:00 # Every midnight
WHENEVER temperature[0] < 22 and (mtime//3600) >= 6 # Every time the temperature is below 22 after 6am
```
Note that at most one instance of a given WHENEVER constraint will be active at a given time.

The body of a WHENEVER constraint indicates a sequence of subconstraints that must hold for the WHENEVER constraint to be successful.  If one of the subconstraints fails to hold, then the WHENEVER constraint fails and a failure message is printed out.  If all of the subconstraints hold, then the WHENEVER constraint succeeds and a success message is printed out.  In either case, the constraint is deactivated (awaiting to be triggered again).  The subconstraints are described below, along with several examples.

##### WAIT #####
The WAIT subconstraint will wait a certain amount of time for a condition to evaluate to true. If the condition is true then this subconstraint succeeds and control is passed to the next one (if any or, if not, the whole WHENEVER constraint succeeds).  If the amount of time passes without the condition being true, then the subconstraint (and the whole WHENEVER constraint) fails.  As with other constraints, the time can be specified as a number of seconds or as a date-time:
```
WAIT temperature[0] < 25 FOR 3600 # Wait an hour for the temperature to come below 25 C
WAIT not led UNTIL 1-23:00:00 # Wait until 11pm for the LEDs to be turned off
```
In addition, a variant of the WAIT constraint can be used without a condition:
```
WAIT FOR 60 # Wait a minute before going on to the next subconstraint.
```
This variant always succeeds, after the given amount of time has passed.

##### ENSURE #####
The ENSURE subconstraint will ensure that some condition is true throughout the whole time period. If the condition is ever false then this subconstraint fails (as does the whole WHENEVER constraint).  If the amount of time passes and the condition remains true during the whole time period, control is passed to the next subconstraint (if any or, if not, the whole WHENEVER constraint succeeds).  As with other constraints, the time can be specified as a number of seconds or as a date-time:
```
ENSURE smoist[0] < 600 and smoist[1] < 600 FOR 3600 # Don't let things get too wet
ENSURE not led UNTIL 2-06:59:59 # Lights must be off until just before 7am the next day 
```

##### SET #####
The SET constraint enables you to specify local variables that can be included in the constraint conditions.  This provides the ability, for instance, to count the number of camera images taken during the day, or how much the soil moisture has changed since the last time the constraint was run.  The general form is "SET <variable> = <value>", where "value" can be a combination of numbers, sensor values, and other variables. The constraint always succeeds.
```
SET last_humid = (humidity[0] + humidity[1])/2
SET dhumid = last_humid - (humidity[0] + humidity[1])/2
SET num_pics = num_pics + 1
```

Simple test examples are provided in the TerraBot/param directory.  Here is a test, consisting of two WHENEVER constraints, for the behavior expected of the pump - that it should turn on soon after the soil moisture falls below some threshold and should not overwater the plants: 
```
WHENEVER smoist[0] < 450 or smoist[1] < 450
  WAIT wpump FOR 60 # Wait one minute for water pump to be on
  WAIT not wpump FOR 360 # Turn pump off before 6 minutes have elapsed
  WAIT smoist[0] > 450 and smoist[1] > 450 FOR 3600 # Wait an hour for both moisture sensors to be above threshold

# Don't let pump overwater things
WHENEVER wpump 
  ENSURE smoist[0] < 600 and smoist[1] < 600 FOR 3600
```

##### PRINT #####
The PRINT constraint enables you to print out information, useful for debugging the testing constraints.  The syntax is like the **print** statement in Python, except without parentheses.  You can use any of the variables that are allowable in WHENEVER, WAIT, and ENSURE statements (including local variables defined using SET).  You can print out the time in string form using **clock_time(time)**.
```
PRINT "W1: %s %s %s" %(wlevel, (wlevel_start - wlevel), wpump_today)
PRINT "Current temperature at %s: %d %d" %(clock_time(time), temperature[0], temperature[1])
```
