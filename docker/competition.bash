#!/usr/bin/env bash

if [ "$#" -ne 1 ]; then
    echo -e "\nUsage: \n # $0 <submission-label>\n"
    exit 1
fi

IMAGE_NAME="metrics2021:$1"

USERID=$(id -u)
GROUPID=$(id -g)

# Run script as per guidelines document
docker run -it \
    --privileged \
    --net=host \
    --env=DISPLAY \
    --volume=$HOME/.Xauthority:/home/metrics/.Xauthority \
    --device /dev/nvidia0:/dev/nvidia0 \
    --device /dev/nvidiactl:/dev/nvidiactl \
    --device /dev/nvidia-uvm:/dev/nvidia-uvm \
    $IMAGE_NAME
