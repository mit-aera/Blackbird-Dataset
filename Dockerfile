FROM winterg/flightgoggles_ros:latest

# Use bash for commands
SHELL ["/bin/bash", "-c"]
WORKDIR /root/

ENV CATKIN_WS /root/catkin_ws
ENV BB_TOOLS_DIR /root/blackbirdDatasetTools
ENV BB_DATA_DIR /root/blackbirdDatasetData

# Install Blackbird Dataset Toolchain.
COPY requirements.txt ${BB_TOOLS_DIR}/

# Install Python 3.7 and set as default for python3 command
RUN apt update \
    && apt install -y software-properties-common \
    && add-apt-repository ppa:deadsnakes/ppa -y \
    && apt update \
    && apt install -y python3.7 python3-pip vim tmux \
    && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.7 2 \
    && pip3 install -r ${BB_TOOLS_DIR}/requirements.txt \
    && apt clean

# install Python prereqs
# RUN pip3 install -r ${BB_TOOLS_DIR}/requirements.txt 
    
# Script files
COPY ./ ${BB_TOOLS_DIR}/

# Copy ROS packages into catkin workspace
COPY ros_utilities/ ${CATKIN_WS}/src/blackbirdDataset

# Catkin build
RUN cd ${CATKIN_WS} \
    && catkin clean -y \
    && source /opt/ros/kinetic/setup.bash \
    && catkin config --cmake-args -DFLIGHTGOGGLES_DOWNLOAD_BINARY=OFF -DCMAKE_BUILD_TYPE=Release \
    && catkin build

# Allow for incoming ports from FG
EXPOSE 10253/tcp
EXPOSE 10254/tcp