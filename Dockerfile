FROM winterg/flightgoggles_ros:latest

# Use bash for commands
SHELL ["/bin/bash", "-c"]
WORKDIR /root/

ENV CATKIN_WS /root/catkin_ws
ENV BB_TOOLS_DIR /root/blackbirdDatasetTools
ENV BB_DATA_DIR /root/blackbirdDatasetData

# Install Blackbird Dataset Toolchain.
COPY requirements.txt ${BB_TOOLS_DIR}/

# Prereqs
RUN apt update \
    && apt install -y python3 python3-pip vim tmux \
    && pip3 install -r ${BB_TOOLS_DIR}/requirements.txt \
    && apt clean

# Script files
COPY ./ ${BB_TOOLS_DIR}/

# Copy ROS packages into catkin workspace
COPY ros_utilities/ ${CATKIN_WS}/src/blackbirdDataset

# Catkin build
RUN cd ${CATKIN_WS} \
    && catkin build

# Allow for incoming ports from FG
EXPOSE 10253/tcp
EXPOSE 10254/tcp