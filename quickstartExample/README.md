# Quickstart Example

This folder contains scripts for downloading and playing back a sample sequence from the Blackbird Dataset. 
This quickstart example assumes that the user has Docker installed, but can be adapted for use with a local ROS install.


```bash
# Set storage location on host computer for a portion of the dataset.
HOST_STORAGE_DIR="$HOME/BlackbirdDatasetData"
# Define this env variable for simplicity of commands
BB_DATA_DIR="/root/BlackbirdDatasetData"

# Download & open a bash terminal in quickstart docker image
docker run -it --rm \
    -v "$HOST_STORAGE_DIR":"$BB_DATA_DIR" \
    -p 10253-10263:10253-10263 \
    winterg/flightgoggles_ros:ijrr \
    /bin/bash

##### In the docker terminal.
# Download a pre-rendered sequence (env variables are pre-populated by Dockerfile)
FLIGHT="ampersand/yawForward/maxSpeed2p0/"
ENVIRONMENT="Small_Apartment"
$BB_TOOLS_DIR/fileTreeUtilities/sequenceDownloader.py --flight=$FLIGHT --environment=$ENVIRONMENT --datasetFolder=$BB_DATA_DIR

# Start playback of the sequence
cd CATKIN_WS/src 
roslaunch blackbird_dataset playback_sequence.launch \
    flight:=$FLIGHT \
    environment:=$ENVIRONMENT \
    datasetDir:=$BB_DATA_DIR
```