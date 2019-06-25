# Three node work station

******

## Running code
first in any open terminal window remember to 'source devel/setup.bash'.
this will allow you to use ros terminal comands.

### Arduino
> *requires roslib Arduino library and SimpleDHT*

## Installing Dependancies
> Side note if you do not have a Sketchbook folder, make one
> To allow uploading to an Arduino 'sudo usermod -a -G dialout $USER'
### Roslib
> 'sudo apt install ros-melodic-rosserial-arduino'  
> 'sudo apt install ros-melodic-rosserial'
Then from sketchbook folder
> 'rosrun rosserial\_arduino make\_libraries.py .'  
### SimpleDHT
From the sketchbook folder  
git clone https://github.com/winlinvip/SimpleDHT.git  
### Arduino.mk
> 'sudo apt install arduino-mk'

Make sure the arduino is connected to the computer first.
Then, check to find make sure that the Makefile has the correct information.
Specifically check 'USER\_LIB\_PATH' 'ARDUINO\_LIBS'.

Then from the folder containing ArduinoCode.ino run 'make upload clean'.

Finally run 'rosrun rosserial\_arduino serial\_node.py /dev/PORT' where PORT 
the port the arduino is connected to.

### Python
To start the relay and student nodes run 'rosrun three\nodes relay.py' etc.

## Editing the Code

### Style

#### Naming
In order to keep the overal structure of the code organized when adding a new 
sensor/actuator simply add \_raw or \_input to it, keeping it consitent.

#### Order
With highly repetative code it is important to keep the variables in order.
Currently the order for the names is
*Sensors*  
Humidity -(humid)  
Temperature -(temp)  
Light -(light)
Water Level -(level)  
Total Dissolved Solids -(tds)  

*Actuators*
LED -(led)  
Water Pump -(wpump)  
Air Pump -(apump)  
Nutrient Pump -(npump)  

### Consistency

Make sure that topics that are published are also subscribed to and vice versa.
If a new sensor/actuator is added make sure that you are consistent with the naming, 
type, and order across the four files






