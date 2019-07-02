#!/bin/bash

kill $(cat processes.txt)

DATE=$(date)
GREEN='\033[1;32m'
echo -e "${GREEN}Closed on $DATE" >> Log/rosserial.log
echo -e "${GREEN}Closed on $DATE" >> Log/student.log
echo -e "${GREEN}Closed on $DATE" >> Log/relay.log

echo  >> Log/rosserial.log
echo  >> Log/student.log
echo  >> Log/relay.log
