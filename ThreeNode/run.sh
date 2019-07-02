#!/bin/bash

roscore > /dev/null &
CORE=$!
sleep 4
#rosrun rosrun rosserial_arduino serial_node.py /dev/ttyACM0
python relay.py &
RELAY=$!
python student.py &
STUDENT=$!

echo "$CORE $RELAY $STUDENT"> processes.txt
