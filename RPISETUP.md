## Installing Software on Raspberry Pi and Arduino ##

## Install Ubunu ##
* Get rpi imager (rasberrypi.com/software - choose general-purpose OS -> Ubuntu -> Ubuntu Desktop 22.04.5 (64 bit)
* Flash to SD card
* Select the language, layout, etc. options. Set user name (robotanist-admin), hostname (terrabot<x>), and password [GET ADMIN PASSWORD FROM REID]

### Connect to Network ###
* If not done through the setup:
    - Hostname: same as computer name; Address: use `cat /sys/class/net/eth0/address`
    - __Wired:__ https://computing.cs.cmu.edu/help-support/equip-registration - Register a New Machine *__or__* Search, Update and Remove Equipment Support
    - __Wifi:__ use these instructions: https://linuxconfig.org/ubuntu-22-04-connect-to-wifi-from-command-line to set up via command line

###  Set up Logins on the Raspberry Pi ###
* `sudo useradd -m robotanist`
* `sudo passwd robotanist` (password is: TerraBot)
* `sudo usermod -a -G dialout,video,audio robotanist`
* `sudo usermod -a -G dialout,video,audio,robotanist robotanist-admin`
* `sudo chsh -s /bin/bash robotanist`
* You have to logout and back in, or reboot, for the usermod changes to take effect

### Installing software ###
Do as robotanist-admin
* `sudo apt update`
* `sudo apt install git vlc curl tmux libraspberrypi-bin`
* `sudo apt install python3 python3-pip python-is-python3`
* `sudo apt install python3-opencv python3-matplotlib`
* `sudo apt install python3-transitions python3-sklearn python3-pandas`
* `sudo pip install dill sendgrid ortools`
* Optional: `sudo apt install xemacs21` (or your favorite text editor)
* `sudo apt upgrade`

### Enabling SSH ###
* `sudo apt install openssh-server`
* `sudo systemctl enable ssh`
* `sudo systemctl start ssh`
* Make sure the /etc/ssh/sshd_config has `PasswordAuthentication yes`

<!--- if you are finding that you are running into errors, try the following:
* ssh-keygen -t rsa -f /etc/ssh/ssh_host_rsa_key  
* ssh-keygen -t ecdsa -f /etc/ssh/ssh_host_ecdsa_key  
* ssh-keygen -t ed25519 -f /etc/ssh/ssh_host_ed25519_key
--->

### Installing ROS ###
Do as robotanist-admin
* `sudo apt install software-properties-common`
* `sudo add-apt-repository universe`
* `export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')`
* `curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"`
* `sudo dpkg -i /tmp/ros2-apt-source.deb`
* add these lines to the end of the .bashrc file (for both robotanist and robotanist-admin, using `sudo xemacs`):
    - `source /opt/ros/humble/setup.bash`
    - `export TB_DIR=${HOME}/TerraBot`
    - `export PYTHONPATH=${PYTHONPATH}:${TB_DIR}:${TB_DIR}/lib:${TB_DIR}/agents`

### Installing TerraBot Software ###
Switch user to robotanist
* `git clone --branch ros2 https://github.com/reidgs/TerraBot` (use your git name and password)

### Installing Arduino ###
Do as robotanist-admin
* `ln -s /home/robotanist/Terrabot .`
* `sudo apt install build-essential arduino arduino-mk python3-serial`
* `sudo chmod a+wrx /dev/ttyACM0`
* `mkdir -p ~/Sketchbook/libraries; cd ~/Sketchbook/libraries`
* `git clone https://github.com/RobTillaart/dhtnew.git`
* `git clone https://github.com/RobTillaart/HX711.git`
* `cd ~/TerraBot/lib/ArduinoCode`
* `make clean; make upload` [note: may have to change the permissions on ArduinoCode to make them available to robotanist-admin)

<!--
### Installing ortools ###
Do as robotanist-admin
* download cmake-3.2.6 from cmake.org and make it
* `python -m pip install numpy==1.21`
* `python -m pip install ortools`
-->
<!---### Set up Python Libraries ###
* cd /home/robotanist-admin/.local/lib/
* sudo cp -r python3.8/site-packages/*  /usr/lib/python3/dist-packages/.
* sudo rm -rf python3.8
--->

### Set up Camera ###
Very complex - might be an easier way, but didn't find one
* Edit /boot/firmware/config.txt to add the following lines:
    - `start_x=1`
    - `gpu_mem=128`
    - `dtoverlay=imx219`
* `sudo update-initramfs -c -k $(umami -r)`
* `sudo apt install ffmpeg ninja-build libboost-all-dev`
* `sudo apt install libexif-dev libjpeg-dev libtiff-dev libv4l-dev libpng-dev`
* `sudo apt install qtbase5-dev qtbase5-dev-tools`
* `sudo apt install python3-jinja2 python3-yaml python3-ply`
* `pip install meson==0.64.1`
* `git clone https://github.com/raspberrypi/libcamera.git`
* `cd libcamera`
* `meson setup build`
* `ninja -C build`
* `sudo ninja -C build install`
* `git clone https://github.com/raspberrypi/rpicam-apps.git`
* `cd rpicam-apps`
* `meson setup build --buildtype=release`
* `ninja -C build`
* `sudo ninja -C build install`
* `wget https://github.com/bluenviron/mediamtx/releases/download/v1.13.0/mediamtx_v1.13.0_linux_arm64.tar.gz`
* `tar -xvzf mediamtx_v1.13.0_linux_arm64.tar.gz`
* `sudo mv mediamtx.yml /etc/.`
* `sudo mv mediamtx /usr/bin/.`
* `rm -rf LICENSE libcamera rpicam-apps`
* `sudo ldconfig`
* put `export rpicam-hello="rpicam-hello --qt-preview -v 0"` in ~/.bashrc
* Reboot

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
