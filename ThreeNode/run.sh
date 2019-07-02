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
rosrun rosserial_arduino serial_node.py /dev/ttyACM0 >> Log/rosserial.log &
ROSARD=$!
python relay.py >> Log/relay.log &
RELAY=$!
python student.py >> Log/student.log &
STUDENT=$!

echo "$RELAY $STUDENT $ROSARD $CORE "> processes.txt
