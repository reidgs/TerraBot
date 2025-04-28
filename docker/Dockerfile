# Basic ROS1 download
FROM registry.hub.docker.com/library/ros:noetic

# install apt-get deps
RUN apt-get update && \
    apt-get install -y git python3 python3-pip python-is-python3 tmux && \
    pip install opencv-python panda3d transitions scikit-learn ortools \
    		matplotlib pandas dill sendgrid

RUN apt-get install -y x11vnc xvfb mesa-utils
EXPOSE 5900
ENV DISPLAY=:1 XRES=1280x800x24

# install ros apt-get deps
RUN apt-get install -y ros-noetic-ros-base && \
    rosdep update

# I like to use this editor
RUN apt-get -y install xemacs21 && apt-get clean

# Create user id
ARG PASSWD=TerraBot
RUN useradd robotanist -m -s /bin/bash -G sudo && \
    echo "robotanist:$PASSWD" | chpasswd
RUN sudo mkdir /tmp/.X11-unix && \
    sudo chown root:robotanist /tmp/.X11-unix && \
    sudo chmod 1777 /tmp/.X11-unix
USER robotanist
WORKDIR /home/robotanist

# install TerraBot software
RUN cd /home/robotanist && \
    git clone https://github.com/reidgs/TerraBot && \
    echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc && \
    echo 'export TB_DIR=${HOME}/TerraBot' >> ~/.bashrc && \
    echo 'export PYTHONPATH=${PYTHONPATH}:${TB_DIR}:${TB_DIR}/lib:${TB_DIR}:${TB_DIR}/agents' >> ~/.bashrc

ENV LIBGL_ALWAYS_SOFTWARE=1 PYOPENGL_PLATFORM=null
RUN echo '#!/bin/sh\n Xvfb $DISPLAY -screen 0 $XRES &\n sleep 1\n \
          x11vnc -display $DISPLAY -N -forever -unixpw > /dev/null 2>&1 &\n \
          exec bash' > ~/.start.sh && \
    chmod a+x ~/.start.sh

# add launcher
CMD ~/.start.sh
