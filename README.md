# Autonomous Agents Terrorium Mk1 #

## Overview ##
Welcome to the Autonomous Agents Terrorium project! For this project you and your partner(s) 
will be given a greenhouse outfitted with multiple sensors and actuators and 10 small pots
with recently sprouted seeds. The goal of the assignment is to provide the best environment
for the plants during a two week grow cycle. Each cycle will come with new challenges, so
be prepared!  

### The System ###
The physical greenhouse you are assigned is only part of the system which you will be 
working with, which is in turn a small part of the system the entire class will working with!  
In control of each box there is a aurdino managing the sensors and actuators and a raspberry
pi where you will run your code. During the beginning of the grow cycle you will be allowed
to upload your code onto your assigned raspberry pi. If, during any point of the grow cycle
you feel as if you must re-upload your code you are free to do so, at a penalty. The point
of this assignment is for the greenhouse to be self sustaining, so it would not make sense
to constantly be changing the program!

### Whats in the box? ###
In order to help maintain a healthy environment for the plants you will need to be able 
to understand the sensor data to determine the current state and activate the actuators
to improve that state when necessary.
We have provided all the sensors and actuators we believe to be neccesary to complete the 
given tasks. These include:  
TODO: format table correctly

| Sensors                     | Message Type | Actuators             | Message Type |
|-----------------------------|--------------|-----------------------|--------------|
| Total disolved solids (tds) | INT32        | LED (led)             | Int32        |
| Current (cur)               | Float32      | Water Pump (wpump)    | Bool         |
| Light (light)               | Int32        | Nutrient Pump (npump) | Bool         |
| Water Level (level)         | Int32        | Air Pump (apump)      | Bool         |
| Temperature (temporary)     | Int32        | Fan (fan)             | Bool         |
| Humidity (humid)            | Int32        |                       |              |



## Understanding ROS ##
In order for you to communicate to and from the machine we will be using the Robot Operating System
(ROS). For the purposes of this assignment we will only be scratching the surface of what ROS is 
capable of. We will be Using ROS mostly for its messaging features. Each sensor and actuator will
send information over a ROS topic of a specific type (shown above). In order to work with the system
your code will create a ROS node which subscribes to each of the sensors topics and publishes to 
actuators.  
TODO Make a helpful visual  
The Terrorium consists of three nodes, one for an arduino, one for a relay to translate data,
and one that you will include. In order to get your code working with the ROS messaging system
follow the tutorial on their web-site [here](TODO:findURL.com). You may find the other tutorials 
there helpful as well! Be sure to check your code to make sure that it is publishing and subscribing
as you intend when bug fixing.
## Setting up the virtual machine ##
Because of the nature of the assignment it is impossible to run any tests of your code before
or between grow cycles. In response you could simply change your code once it is on the machines,
but each time you do you will be penalized! In order to avoid those penalties and allow you to
test your code, we have included a simulator which closely mimics an actual greenhouse. In order
to run a simulated version of the system you will need to have ROS installed on your 
own machine. Because of the OS restrictions we believe that the best way to do this is with a
ubuntu virtual machine.

### Installing Virtual Box ###
Follow the instructions [here](https://www.wikihow.com/Install-VirtualBox) for installing 
VirtualBox, a software which help you easily create virtual machines.

### Downloading the Operating system ###
We recommend that you download ubuntu 32 bit [here](https://ubuntu-mate.org/download/) to most
accurately mimic the experience that you will have on the pi. (any ubuntu distro should be valid
though this is the only one we have tested)

### Downloading the Simulator ###
TODO Find out how students will get the simulator

### Downloading ROS ###
On an ubuntu virtual machine please follow the instruction
[here](https://wiki.ros.org/melodic/Installation/Ubuntu_) for installing ROS.
Keep in mind that only the Desktop install is neccesary. Depending on your internet connection
this step may take a while.

## Running the Simulator ##

## Uploading to the Greenhouse ##

## Grading ##

