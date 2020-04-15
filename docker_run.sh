#!/bin/bash
# Set storage location on host computer for a portion of the dataset.
HOST_STORAGE_DIR="$HOME/blackbirdDatasetData"
# Define this env variable for simplicity of commands
BB_DATA_DIR="/root/blackbirdDatasetData"
# Download & open a bash terminal in quickstart docker image
docker run -it --rm \
    -v $HOST_STORAGE_DIR:$BB_DATA_DIR \
    -e DISPLAY=host.docker.internal:0 \
    -p 10253-10263:10253-10263 \
    winterg/flightgoggles_ros:ijrr \
    /bin/bash