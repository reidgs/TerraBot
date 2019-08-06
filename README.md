# Autonomous Agents TerraBot Mk1 #

- [Autonomous Agents TerraBot Mk1](#autonomous-agents-terrabot-mk1)
  - [Overview](#overview)
    - [TerraBot Software Architecture](#terrabot-software-architecture)
  - [ROS Communication](#ros-communication)
  - [Understanding the System](#understanding-the-system)
    - [Student Node](#student-node)
    - [Relay Node](#relay-node)
        + [Interference File](#interference-file)
    - [Arduino Node](#arduino-node)
  - [Getting Started](#getting-started)
    - [TerraBot Simulator Installation](#terrabot-simulator-installation)
  - [Running the Simulator](#running-the-simulator)
    - [Connecting to the Raspberry Pi](#connecting-to-the-raspberry-pi)
    - [Uploading Code](#uploading-code)
  - [Deployment and Testing](#deployment-and-testing)
    - [Health Ping](#health-ping)
    - [Frequency](#frequency)
    - [Time](#time)
    - [Camera](#camera)

## Overview ##

Welcome to the Autonomous Agents TerraBot project! For this project you and your partner(s)
will be given a greenhouse outfitted with multiple sensors and actuators and 10 small pots
with recently sprouted seeds. The goal of the assignment is to provide the best environment
for the plants during a two week grow cycle. Each cycle will come with new challenges, so
be prepared!  

| Name                         | Description                                                 | Message Type | Range      |
| ---------------------------- | ----------------------------------------------------------- | ------------ | ---------- |
| Sensors                      | These Sensors allow the user to determine the systems state |              |            |
| Total Dissolved Solids (tds) | EC of water (useful for nutrient monitoring)                | Int32        |            |
| Current (cur)                | The current draw of the system.                             | Float32      |            |
| Light (light)                | Light in the system                                         | Int32        |            |
| Water level (level)          | The current hight of the water                              | Int32        |            |
| Temperature (temp)           | Internal temperature of the system                          | Int32        |            |
| Humidity (humid)             | Internal relative humidity                                  | Int32        |            |
| Camera                       | Allows to students to photograph the system                 | None         |            |
| Actuators                    | These Actuators allow the user to adjust the systems state  |              |            |
| LED (led)                    | Adjust the power going to the systems LED light fixture     | Int32        | 0-255      |
| Water Pump (wpump)           | Toggle whether the water pump is on or off                  | Bool         | true-false |
| Nutrient Pump (npump)        | Toggle whether the nutrient pump is on or off               | Bool         | true-false |
| Air Pump (apump)             | Toggle whether the air pump is on or off                    | Bool         | true-false |
| Fan (fan)                    | Toggle whether the fan is on or off                         | Bool         | true-false |

//TODO QUESTION - SHOULD WE PUBLISH A MESSAGE OF THE CURRENT ACTUATOR STATE? IF THINGS CRASH AND THE LEDs ARE ON, HOW DOES THE AGENT KNOW THAT?

### TerraBot Software Architecture ###

An arduino communicates directly with these sensors and actuators and forwards that data to a raspberry pi.
The raspberry pi, running ROS (Robot Operating System), receives the sensor data, cleans it, and makes it available
for your AI agent in the formats above. Additionally, it receives your agent's actuator commands as defined above
and relays them back to the arduino.

![system diagram](https://user-images.githubusercontent.com/37941547/62577548-2a2c4080-b86d-11e9-9571-c5b06707976a.jpg)

The above image shows an overview of the connections between the different nodes (represented by ovals) in the system.
Notice all the topics (represented by rectangles) connected to the student, as those are the ones you will be using to
regulate your greenhouse.

## ROS Communication ##

In order to get your code working with the ROS messaging system,
follow the tutorial on the ROS website [here](https://wiki.ros.org/ROS/Tutorials).
Please look over the tutorials concerning ROS communication (Nodes, Topics, Publishers + Subscribers) to gain a general understanding of ROS.
You may find the other tutorials there helpful as well.


The TerraBot consists of three ROS *nodes* or processes, one for the arduino communication with the raw data,
one hardware feed for publishing the clean data and listening for actuation commands,
and one that you will write for your agent.
The hardware feed *publishes* sensor data and *subscribes* to actuation commands over ROS topics
of specific types (shown above).
Your agent will be a ROS *node* which subscribes to each of the sensors topics, plans actions, and publishes to
actuators.
 
Be sure to check
your code to make sure that it is publishing and subscribing as you intend when bug fixing.

## Understanding the System ##

As mentioned earlier, there are three ROS nodes in this system: the student, which you will provide, the relay,
and the arduino.

### Student Node ###
The student node is your reactive agent. You will be able to access sensor data by subscribing to the topics to which the relay publishes. You will also be able to write data to the actuators by publishing to the topics to which the relay subscribes.

### Relay Node ###
Running the relay will start up a total of 4 nodes. First the master node, roscore, and then the three other nodes: student, relay and Arduino. 

The relay transfers actuator data from the student node to the Arduino node:
The relay subscribes to the topics to which the student node publishes. It also publishes to the topics to which the Arduino subscribes.

The realy also transfers sensor data from the Arduino node to the student node:
The relay subscribes to the topics to which the Arudino publishes. It also publishes to the topics to which the studnet subscribes.
 
In the transfering process, the data received by the relay node are passed though functions via an external interference file. In order to reliably simulate errors which may happen by chance if run in the real world, the interference file
may be malicious and cause the relay to act incorrectly.

#### Interference File ####
The interference file may take in a path to a .txt file containing a schedule of times to interfere with data. There are three types of functions, through one of which your data will be passed: normal, noise, and off. 

* normal : trasfers data directly without any modifications
* noise : modifies data before transfering
* off : sets all data to 0 (or type equivelent) 

*If no file is passed in, there will be no intereference in the transfer of data.*


### Arduino Node ###
Sensors and actuators are being controlled in the Arduino node:

* The Arduino reads in all sensor data and translates them from raw values to more meaningful values that are then published to the Relay node. 
* The Arduino also subscribes to topics containing data values that are published by the Relay node. These meaningful values are translated to its raw form, with which the Arduino can write to the actuators.

All communication to and from the arduino is done via the relay node, meaning you should
never access the same topics as the Arduino. 

## Getting Started ##

We are providing a simulator to test your code before deploying it on the TerraBot. Follow the instructions to get
started with the simulator and also to log into your raspberry pi.

### TerraBot Simulator Installation ###

The simulator and ROS require Ubuntu distributions. We suggest installing a VirtualBox VM on your computer so that you
can implement your agent.

- VirtualBox: [Instructions here](https://www.wikihow.com/Install-VirtualBox) for installing
VirtualBox.
- Ubuntu: download ubuntu 32 bit [here](https://ubuntu-mate.org/download/) to most
accurately mimic the software on the pi. (any ubuntu distro should be valid though this is the only one we have tested)
- ROS: [Instructions here](https://wiki.ros.org/melodic/Installation/Ubuntu_) for installing ROS.
Keep in mind that only the Desktop install is neccesary. Depending on your internet connection
this step may take a while.
- TerraBot Simulator
QUESTION - I THINK WE CAN ZIP UP THE WHOLE OS, WILL THESE INSTRUCTIONS CHANGE FOR THAT?
//TODO Find out how students will get the simulator

## Running the Simulator ##

The simulator works in a way almost identical to the three node process which will run when your
code is uploaded to the raspberry pi. The code for your node and the relay node is the exact same
as it would be on the pi. Instead of having an arduino node, however, the simulator comes with a
farduino (fake arduino) node which mimics the actions of the arduino node. This difference should
in no way affect the way your code operates and should not be noticeable from the perspective
of your node.

In order to run the simulator, run the relay.py with the -s flag, the multiplier you wish for the speed,
and the time which you would like it to start at (seconds since epoch).  
For example if I wanted to run the simulator at 1x speed at time=0 I would run:  
>`python relay.py -s 1 0`  
For error checking it is recommended that you include the -l flag for logging as well.  
EX: 5x speed with logging
>`python relay.py -l -s 5 0`  

### Connecting to the Raspberry Pi ###

//TODO INCLUDE THE INSTRUCTIONS FOR GETTING STARTED WITH THE ACTUAL PI, WHERE IS CODE LOCATED, WHERE SHOULD THEY MOVE THEIR CODE, HOW DO THEY RUN IT, ETC

### Uploading Code ###

## Deployment and Testing ##

In order to allow for greater control of the system and to ensure the accuracy of the simulator,
there are a few extra processes to which you have access. In addition to the previously mentioned sensors
and actuators, there will also be a health ping, variable time speed, and a frequency topic which you must consider.

### Health Ping ###

Because of the long lasting nature of this project, it is possible that there may be unforeseen
errors in your code which will cause it to crash. Crashed code means no control over the system and
certain doom for your plants! In order to avoid this outcome, we have included restart functionality.
When the relay begins, it will run your code and listen for a ping. If your ping is not heard within
a set amount of time (default 60 min), it will assume your program has crashed and restart it automatically.

### Frequency ###

The frequency topic is used to determine how often the arduino will read from the sensors.
The more often you read, the more accurate your data will be, but the more power you will draw as well.
Notice that this setting is variable, meaning it can be changed over the course of the deployment.

### Time ###

One of the most convenient aspects of the simulator is its ability to manipulate time in order to
suit the user's needs. By default the simulator will begin running at 1x speed at the epoch,
but that can be configured with the -s flag.

It is also important that the execution of the simulator is identical to the relay (even if sped up).
**To ensure consistency between your code in simulation and on TerraBot, you should refrain from referring to outside functions
(OS time.time()) and should instead refer to the ROS time topic via rospy.get_time().**

### Camera ###

The one aspect of the system which we are not able to simulate is the camera. Any call to
raspistill will result in an error as there is no camera connected to the virtual machine
and raspistill is not installed.

//TODOCAN WE RENAME raspistill? It looks like rapist...
