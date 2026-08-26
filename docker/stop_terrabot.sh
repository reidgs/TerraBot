#!/bin/bash

CONTAINER_NAME=terrabot_container

# Check if the container exists
if docker ps --format '{{.Names}}' | grep -Eq "^${CONTAINER_NAME}\$"; then
    echo "Stopping $CONTAINER_NAME"
    docker container stop $CONTAINER_NAME
    docker container rm $CONTAINER_NAME
else
    echo "$CONTAINER_NAME not currently running"
fi
