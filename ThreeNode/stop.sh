#!/bin/bash

kill $(cat processes.txt)
rm processes.txt
rm *.pyc
sleep 3
DATE=$(date)
GREEN='\033[1;32m'
NC='\033[0m'
echo -e "${GREEN} Closed on $DATE ${NC}\n" >> Log/rosserial.log
echo -e "${GREEN} Closed on $DATE ${NC}\n" >> Log/student.log
echo -e "${GREEN} Closed on $DATE ${NC}\n" >> Log/relay.log
