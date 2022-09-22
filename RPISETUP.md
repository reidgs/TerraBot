## Installing Software on Raspberry Pi and Arduino ##

## Install Ubunu ##
* Download Ubuntu (ubuntu-mate-20.04.1-desktop-amd64.iso from https://releases.ubuntu.com)
* Use balenaEtcher to write to the SD card (desktop is not 100% needed, but makes setup a little easier)

###  Set up Logins on the Raspberry Pi ###
* Name computer terrabot1 (or whichever number)
* Make user name be robotanist-admin, with passphrase: [GET ADMIN PASSWORD FROM REID]
* Menu -> Users + Groups
* Add user robotanist, password TerraBot; log in automatically

### Creating Swap File ###
Do as robotanist-admin;
Check whether already have a swap file: cat /proc/swaps; if not:
* sudo apt install dphys-swapfile`
* edit /etc/dphys-swapfile and uncomment CONF_SWAPFILE line
* sudo dphys-swapfile setup
* sudo dphys-swapfile swapon

### Enabling SSH ###
* sudo systemctl enable ssh  
* sudo systemctl start ssh

if you are finding that you are running into errors, try the following:
* ssh-keygen -t rsa -f /etc/ssh/ssh_host_rsa_key  
* ssh-keygen -t ecdsa -f /etc/ssh/ssh_host_ecdsa_key  
* ssh-keygen -t ed25519 -f /etc/ssh/ssh_host_ed25519_key

### Installing software ###
Do as robotanist-admin
* sudo apt update
* sudo apt install python3 python3-pip git
* sudo apt install python-is-python3
* pip3 install transitions sklearn 
* sudo apt install vlc python3-opencv matplotlib tmux
* Optional: sudo apt install xemacs21 (or your favorite text editor)

### Installing ROS ###
Do as robotanist-admin
* sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
* sudo apt install curl
* curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
* sudo apt install build-essential arduino arduino-mk 
* sudo apt install ros-noetic-rosserial ros-noetic- rosserial-arduino
* add the line to the end of the .bashrc file
    - source /opt/ros/noetic/setup.bash

### Installing TerraBot Software ###
Do as robotanist
* cd Desktop; git clone https://github.com/reidgs/TerraBot (use your git name and password)
* cd $HOME; ln -s Desktop/TerraBot .
* add the line “source /opt/ros/noetic/setup.bash” to the end of the .bashrc file; source ~/.bashrc
* add the lines to bashrc (for both robotanist and robotanist-admin):
    - export TB_DIR=${HOME}/Desktop/TerraBot 
    - export PYTHONPATH=${PYTHONPATH}:${ TB_DIR}:${ TB_DIR}/lib:${ TB_DIR}:${ TB_DIR}/agents
* invoke raspi-config and enable camera and SSH

### Installing Arduino ###
Do as robotanist-admin
* cd Desktop; ln -s /home/robotanist/Desktop/Terrabot .
* cd $HOME; ln -s Desktop/TerraBot .
* mkdir ~/Sketchbook/libraries; cd ~/Sketchbook/libraries
* rosrun rosserial_arduino make_libraries.py .
* git clone https://github.com/RobTillaart/DHT20.git
* sudo usermod -a -G dialout,robotanist $USER
* sudo usermod -a -G dialout robotanist
* cd ~/TerraBot/lib/ArduinoCode
* make clean; make upload [note: may have to change the permissions on ArduinoCode to make them available to robotanist-admin)

### Installing ortools ###
Do as robotanist-admin
* sudo apt install swig
* cd /usr/lib/python3.8
* sudo git https://github.com/google/or-tools.git
* cd or-tools
* sudo make third_party > $HOME/make_third_party.txt
    - I had to update config.guess and config.sub repeatedly – couldn’t find where they were being taken from, but they were old versions.
* sudo make python
* sudo make test
* cd ortools; sudo cp init.p* /usr/lib/python3.8; sudo cp init.p* /usr/lib/python3.8/or-tools
* add the line to .bashrc (both for robotanist and robotanist-admin) :
    - export PYTHONPATH=${PYTHONPATH}:/usr/lib/python3.8:/usr/lib/python3.8/or-tools

### Set up Python Libraries ###
* cd /home/robotanist-admin/.local/lib/
* sudo cp -r python3.8/site-packages/*  /usr/lib/python3/dist-packages/.
* sudo rm -rf python3.8

### Copy Setup to Other SD Cards ###
* https://computers.tutsplus.com/articles/how-to-clone-your-raspberry-pi-sd-cards-with-windows--mac-59294
* Change the machine name accordingly:
    - sudo hostnamectl set-hostname newNameHere
    - sudo xemacs /etc/hosts – replace all occurrences of existing computer name
* Regenerate ssh server keys
    - sudo rm -v /etc/ssh/ssh_host_*
    - sudo dpkg-reconfigure openssh-server
    - sudo systemctl restart ssh
* Deal with the audio server
    - sudo chown robotanist /usr/bin/pulseaudio
    - sudo chgrp robotanist /usr/bin/pulseaudio
    - pactl list -> look for the card associated with the microphone
    - edit stream-av and stream-audio to change the plughw card to match (e.g., plughw:1,0)
 * Check pump pressure
    - if pump is too weak (or too strong), edit wpump_activate in Arduino.ino (and make upload)


