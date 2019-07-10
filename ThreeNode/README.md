# Three node work station

******

## Running code
To start up the three node code simply run `./run.sh`  
When finished `./stop.sh`  
NOTE: if you do not stop bad things will happen (proccess will not stop)

### Arduino
> *requires roslib Arduino library and SimpleDHT*

## Installing Dependancies
> Side note if you do not have a Sketchbook folder, make one  
> To allow uploading to an Arduino `sudo usermod -a -G dialout $USER`
### Roslib
> `sudo apt install ros-melodic-rosserial-arduino`  
> `sudo apt install ros-melodic-rosserial`  
Then from sketchbook folder
> `rosrun rosserial_arduino make_libraries.py .`  
### SimpleDHT
From the sketchbook folder  
git clone https://github.com/winlinvip/SimpleDHT.git  
### Arduino.mk
> `sudo apt install arduino-mk`  

Make sure the arduino is connected to the computer first.
Then, check to find make sure that the Makefile has the correct information.
Specifically check `USER_LIB_PATH` `ARDUINO_LIBS`.

Then from the folder containing ArduinoCode.ino run `make upload clean`.

Finally run `rosrun rosserial_arduino serial_node.py /dev/PORT` where PORT 
the port the arduino is connected to. (Most likely ttyACM0)

### Python
~~To start the relay and student nodes run `rosrun three\nodes relay.py` etc.~~
No longer using ros ws so simply use python relay.py etc.

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
Current -(cur)  

*Actuators*  
LED -(led)  
Water Pump -(wpump)  
Air Pump -(apump)  
Nutrient Pump -(npump)  
Fan -(fan)  

### Consistency

Make sure that topics that are published are also subscribed to and vice versa.
If a new sensor/actuator is added make sure that you are consistent with the naming, 
type, and order across the four files







