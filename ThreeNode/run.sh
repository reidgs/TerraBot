#!/bin/bash

DATE=$(date)
BLUE='\033[1;34m'
NC='\033[0m'
echo -e "${BLUE} Opened on $DATE ${NC}\n" >> Log/rosserial.log 
echo -e "${BLUE} Opened on $DATE ${NC}\n" >> Log/student.log 
echo -e "${BLUE} Opened on $DATE ${NC}\n" >> Log/relay.log

roscore > /dev/null &
CORE=$!
sleep 4
rosrun rosserial_arduino serial_node.py /dev/ttyACM0 >> Log/rosserial.log 2>&1 &
ROSARD=$!
python relay.py -l -v >> Log/relay.log 2>&1 &
RELAY=$!
python student.py >> Log/student.log 2>&1 &
STUDENT=$!

echo "$RELAY $STUDENT $ROSARD $CORE "> processes.txt
