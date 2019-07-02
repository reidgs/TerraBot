#!/bin/bash

kill $(cat processes.txt)

DATE=$(date)
GREEN='\033[1;32m'
NC='\033[0m'
echo -e "${GREEN} Closed on $DATE ${NC}" >> Log/rosserial.log
echo -e "${GREEN} Closed on $DATE ${NC}" >> Log/student.log
echo -e "${GREEN} Closed on $DATE ${NC}" >> Log/relay.log

echo  >> Log/rosserial.log
echo  >> Log/student.log
echo  >> Log/relay.log
