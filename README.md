# Robotanist #

## Initializing a workspace ##

### First Create the Workspace ###
mkdir -p robotonist_ws/src  
`catkin\_make` in that directory (the parent not the src)  
will make three seperate folders `build, devel, src`  
`source devel/setup.bash` source the setup file  
to make sure everything is working run `echo $ROS\_PACKAGE\_PATH`  
Should return   
    /home/\<usr\>/\<path\>/robotonist\_ws/src:/opt/ros/kinetic/share
### Now we must create a catkin Package ###
To do this we must determine exactly what dependancies  
catkin\_create\_pkg <package\_name> [depend1] [depend2] ...  
Now go back to the ws folder and rerun catkin_make  
`. devel/setup.bash` to source the setup file  
Use rospack depends to determine all dependancies  
### Now we create the script ###
create a publisher and a subscriber  
### Now we run the script ###
Make sure roscore is up and running  
source the setup.bash file `source ./devel/setup.bash`  
rosrun \<package name\> <file.py>  


## Steps to follow on each PIE ##  
Install dependancies  
Install extensions  

### Install dependancies ###

#### ROS ####

First follow the instructions on the ROS site for instalations  
Found **INSTALL MELODIC**[here](http://wiki.ros.org/melodic/Installation/Ubuntu)  
As far as we are concerned only the Desktop install is necesary  
though the base may work (**untested**)  

Follow instructions [here](https://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup) to install rosserial and rosserial ardiuno  
**REMEMBER THAT YOU ARE USING MELODIC**

#### Python ####
install pip `sudo apt install python-pip`  
`pip install:`  

#### Arduino ####
`sudo apt install arduino.mk`  
then use the provided Makefile  
change filepaths as necessary  
