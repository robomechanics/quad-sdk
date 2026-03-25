#!/bin/bash

PROJECT_DIR=$(dirname "$PWD")
cd "$PROJECT_DIR"

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <container_name> <image_name:tag>"
  exit 1
fi
CONTAINER_NAME="$1"
IMAGE_NAME="$2"

docker run --privileged -it \
           --volume="$PROJECT_DIR:/root/ros2_ws/src/quad-sdk" \
           --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
           --net=host \
           --ipc=host \
           --shm-size=4gb \
           --name="$CONTAINER_NAME" \
           --env="DISPLAY=$DISPLAY" \
           --env="ROS_DOMAIN_ID=42" \
           --env="ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET" \
           --rm \
           "$IMAGE_NAME" /bin/bash
