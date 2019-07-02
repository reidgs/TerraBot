#!/bin/bash

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
