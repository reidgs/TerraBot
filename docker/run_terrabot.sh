#!/bin/bash

USER_DIR="${1:-.}"

CONTAINER_NAME=terrabot_container
IMAGE_NAME=terrabot_image
PORT=5901

# Check if the container exists
if docker ps --format '{{.Names}}' | grep -Eq "^${CONTAINER_NAME}\$"; then
    echo "Starting existing container: $CONTAINER_NAME"
    docker exec -it $CONTAINER_NAME bash
else
    echo "Starting new container: $CONTAINER_NAME"
    docker run -it --rm --name $CONTAINER_NAME -p $PORT:$PORT \
               --volume "$USER_DIR":/home/robotanist/User $IMAGE_NAME
fi