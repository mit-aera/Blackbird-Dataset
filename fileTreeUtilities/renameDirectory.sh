#!/bin/bash

# ARGS: $1='./patrick/yawForward/maxSpeed2p0/Camera_L_Ancient_Asia_Museum_Room'


# Extract Camera type
#CAMERA_TYPE=`echo "$1" | grep -o "Camera_."`

# Extract Experiment name
#EXPERIMENT_NAME=`echo "$1" | sed s/.*Camera_._//`

# Create folder experiment/camera
DESTINATION_FOLDER=`echo $1 | sed 's%[^/]*\(Camera_.\)_\(.*\)%\2/\1%'`

# mv contents of original folder to experiment/camera
mkdir -p $DESTINATION_FOLDER
mv $1/* $DESTINATION_FOLDER/

