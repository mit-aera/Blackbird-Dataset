FROM winterg/flightgoggles_ros:latest

# Use bash for commands
SHELL ["/bin/bash", "-c"]
WORKDIR /root/

# Install Blackbird Dataset Toolchain.
COPY requirements.txt /root/blackbird-dataset/

# Prereqs
RUN apt update \
    && apt install -y python3 python3-pip \
    && pip3 install -r /root/blackbird-dataset/requirements.txt \
    && apt clean

# Script files
COPY ./ /root/blackbird-dataset/

# Allow for incoming ports from FG
EXPOSE 10253/tcp
EXPOSE 10254/tcp