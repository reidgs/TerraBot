## Creating VirtualBox Image for TerraBot ##

### Set up VirtualBox Image ### 
In VirtualBox
* Choose “New”; Name: TerraBot; Type: Linux; Version: Ubuntu (64-bit)
* RAM: 4096 MB
* Hard disk: Accept default option, choose “Create”
* Hard disk file type: Accept default option, choose “Next”
* Storage on physical hard disk: either option is fine
* File location and size: Enter 20GB; choose “Create”

### Installing Ubuntu ###
* Download Ubuntu (ubuntu-20.04.2.0-desktop-amd64.iso from https://releases.ubuntu.com)
* Power on (start) TerraBot machine; find the ISO Ubuntu image and start creating machine; follow instructions to install Ubuntu;
    -	“your name” is Robotanist; computer’s name is TerraBot, username is robotanist, password is TerraBot; choose “Log in automatically”
* After installation, restart (**don’t forget to press “Enter”**)

###	Resizing window ###
* In “Devices” menu bar, select “Insert Guest Additions CD Image” and choose “run”
* Enter password and choose “Authenticate”; press return when completed
* Right-click on disk and eject it
* Restart the machine
* Change window to desired size and then in “View” menu, select “Auto resize guest display”

###	Installing software ###
* sudo apt install python3 python3-pip git
* sudo apt install python-is-python3
* pip install panda3d transitions sklearn ortools matplotlib
* sudo apt install python3-opencv
* **Optional:** sudo apt install xemacs21 (or your favorite text editor)
* **Optional:** in the applications box, search for and install PyCharm CE

###	Installing ROS ###
* sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
* sudo apt install curl # if you haven't already installed curl
* curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
* sudo apt update
* sudo apt install ros-noetic-ros-base
* add the line to the end of the .bashrc file
    - source /opt/ros/noetic/setup.bash

### Installing TerraBot ###
* cd Desktop; git clone https://github.com/reidgs/TerraBot (use your git name and password)
* add the lines to bashrc:
    - export TB_DIR=${HOME}/Desktop/TerraBot 
    - export PYTHONPATH=${PYTHONPATH}:${ TB_DIR}:${ TB_DIR}/lib:${ TB_DIR}:${ TB_DIR}/agents

### Create OVA ###
* In VirtualBox, File -> Export Appliance
* Be sure to choose the location for writing the file; otherwise accept all the defaults

### Optimize Parameters ###
* Choose **Settings -> System -> Motherboard**
* Set RAM to about half of your total RAM (up to 10GB)
* Choose **Processor** tab
* Set it to at least 4 (more is better, but also keep in mind how many CPU threads you have -- at least a couple should be left for your overall machine not given to the submachine)
* Click **OK**




