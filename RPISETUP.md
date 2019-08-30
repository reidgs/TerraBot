# Setting RPI #

## Installing OS ##
[here](https://ubuntu-mate.org/download/)  
Use balenaEtcher to write to the sd card  
(desktop is not 100% needed but makes setup a little easier)


## Setting up ##

Connect to CMU Secure (WPA2 Enterprise and PEAP security) No certificate
needed. Use you Andrew ID and password. 

Name the computer AS-House-1 (or whichever number house).  
Make password robotanist

Log in automatically

`sudo apt update`  
`raspi-config` and enable camera and SSH

### SSH ###

`sudo systemctl enable ssh  
sudo systemctl start ssh`

if you are finding that you are running into errors

`ssh-keygen -t rsa -f /etc/ssh/ssh_host_rsa_key  
ssh-keygen -t ecdsa -f /etc/ssh/ssh_host_ecdsa_key  
ssh-keygen -t ed25519 -f /etc/ssh/ssh_host_ed25519_key`  
may be necessary.

### Installing Software ###
install ROS from [here](https://wiki.ros.org/melodic/Installation/Ubuntu)
(easiest to do over SSH)
sudo apt install the following packages
| Name                          |
|-------------------------------|
| build-essential               |
| git                           |
| arduino                       |
| arduino.mk                    |
| ros-melodic-rosserial         |
| ros-melodic-rosserial-arduino |

## Arduino ##
First create the sketchbook folder which the arduino will reference.  
`mkdir ~/Documents/Sketchbook`  
From the Sketchbook folder run  
`rosrun rosserial_arduino make_libraries.py .`  
`git clone https://github.com/winlinvip/SimpleDHT.git`  
Then to allow arduino to use the usb port run
`sudo usermod -a -G dialout $USER`  

## Source code ##
Install the source code from github  
`git clone https://github.com/JackMenandCameron/RoBotanist.git`  
Remember it is a restricted repository so you need to have permission.  
for consistency sake download it into the documents folder as well.

Make sure that the code in the Make file for the ArduinoCode is correct.
(check that USER\_LIB_PATH = /home/as-house-1/Documents/Sketchbook)
To make the code, in lib/ArduinoCode:
	make clean; make
To upload the code to the Arduino
	make upload



