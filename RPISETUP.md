## Installing Software on Raspberry Pi and Arduino ##

## Install Ubunu ##
* Get rpi imager (rasberrypi.com/software - choose general-purpose OS -> Ubuntu -> Ubuntu Server 20.04.5 (64 bit)
* Choose "Edit Settings" and set hostname (terrabot<x>), user name (robotanist-admin), and passphrase [GET ADMIN PASSWORD FROM REID]
* Flash to SD card

### Connect to Network ###
* Hostname: same as computer name; Address: use `cat /sys/class/net/eth0/address`
* __Wired:__ https://computing.cs.cmu.edu/help-support/equip-registration - Register a New Machine *__or__* Search, Update and Remove Equipment Support
* __Wifi:__ use these instructions: https://linuxconfig.org/ubuntu-20-04-connect-to-wifi-from-command-line to set up via command line

### Install desktop (not 100% needed, but makes setup a lot easier) ###
* `sudo apt update & sudo apt upgrade`
* `sudo apt install ubuntu-desktop`
* `sudo reboot`

###  Set up Logins on the Raspberry Pi ###
* `sudo useradd -m robotanist`
* `sudo passwd robotanist` (password is: TerraBot)
* `sudo usermod -a -G dialout,video,audio,robotanist robotanist-admin`
* `sudo usermod -a -G dialout,video,audio robotanist`

### Creating Swap File ###
Do as robotanist-admin;
Check whether already have a swap file: cat /proc/swaps; if not:
* `sudo apt install dphys-swapfile`
* edit /etc/dphys-swapfile and uncomment CONF_SWAPFILE line
* `sudo dphys-swapfile setup`
* `sudo dphys-swapfile swapon`


### Enabling SSH ###
* `sudo systemctl enable ssh`
* `sudo systemctl start ssh`

* Make sure the /etc/ssh/sshd_config has `PasswordAuthentication yes`

<!--- if you are finding that you are running into errors, try the following:
* ssh-keygen -t rsa -f /etc/ssh/ssh_host_rsa_key  
* ssh-keygen -t ecdsa -f /etc/ssh/ssh_host_ecdsa_key  
* ssh-keygen -t ed25519 -f /etc/ssh/ssh_host_ed25519_key
--->

### Installing software ###
Do as robotanist-admin
* `sudo apt update`
* `sudo apt install git vlc curl`
* `sudo apt install python3 python3-pip python-is-python3`
* `sudo apt install python3-opencv python3-matplotlib tmux`
* `sudo apt install python3-transitions python3-sklearn`
* `sudo apt install libraspberrypi-bin`
* Optional: `sudo apt install xemacs21` (or your favorite text editor)

### Installing ROS ###
Do as robotanist-admin
* `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
* `curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
* `sudo apt update`
* `sudo apt install build-essential arduino arduino-mk`
* `sudo apt install ros-noetic-rosserial ros-noetic-rosserial-arduino`
* add the line to the end of the .bashrc file
    - `source /opt/ros/noetic/setup.bash`

### Installing TerraBot Software ###
Switch user to robotanist
* `cd Desktop; git clone https://github.com/reidgs/TerraBot` (use your git name and password)
* `cd $HOME; ln -s Desktop/TerraBot .`
* add the line `source /opt/ros/noetic/setup.bash` to the end of the .bashrc file
* add the lines to bashrc (for both robotanist and robotanist-admin):
    - `export TB_DIR=${HOME}/Desktop/TerraBot`
    - `export PYTHONPATH=${PYTHONPATH}:${TB_DIR}:${TB_DIR}/lib:${TB_DIR}/agents`
* `source ~/.bashrc`
  
### Installing Arduino ###
Do as robotanist-admin
* `cd Desktop; ln -s /home/robotanist/Desktop/Terrabot .`
* `cd $HOME; ln -s Desktop/TerraBot .`
* `mkdir ~/Sketchbook/libraries; cd ~/Sketchbook/libraries`
* `rosrun rosserial_arduino make_libraries.py .`
* `git clone https://github.com/RobTillaart/dhtnew.git`
* `git clone https://github.com/RobTillaart/HX711.git`
* `cd ~/TerraBot/lib/ArduinoCode`
* `make clean; make upload` [note: may have to change the permissions on ArduinoCode to make them available to robotanist-admin)

### Installing ortools ###
Do as robotanist-admin
<!-- * download cmake-3.2.6 from cmake.org and make it -->
* `python -m pip install numpy==1.21`
* `python -m pip install ortools`

<!---### Set up Python Libraries ###
* cd /home/robotanist-admin/.local/lib/
* sudo cp -r python3.8/site-packages/*  /usr/lib/python3/dist-packages/.
* sudo rm -rf python3.8
--->

### Set up Camera ###
* `df | grep firmware` => use this device name below
* `sudo mount /dev/mmcblk0p1 /boot`
* Edit /boot/firmware/config.txt to add the following lines:
    - `start_x=1`
    - `gpu_mem=128`
* Reboot (and check using raspivid -t 1000)

### Copy Setup to Other SD Cards ###
* https://computers.tutsplus.com/articles/how-to-clone-your-raspberry-pi-sd-cards-with-windows--mac-59294
* Change the machine name accordingly:
    - `sudo hostnamectl set-hostname newNameHere`
    - `sudo xemacs /etc/hostname` – replace occurrence of existing computer name
    - `sudo xemacs /etc/hosts` – replace all occurrences of existing computer name
* Regenerate ssh server keys
    - `sudo rm -v /etc/ssh/ssh_host_*`
    - `sudo dpkg-reconfigure openssh-server`
    - `sudo systemctl restart ssh`
* Deal with the audio server
    - `sudo chown robotanist /usr/bin/pulseaudio`
    - `sudo chgrp robotanist /usr/bin/pulseaudio`
    - `pactl list` -> look for the card associated with the microphone
    - edit stream-av and stream-audio to change the plughw card to match (e.g., plughw:1,0)
 * Check pump pressure
    - if pump is too weak (or too strong), edit wpump_activate in Arduino.ino (and make upload)


