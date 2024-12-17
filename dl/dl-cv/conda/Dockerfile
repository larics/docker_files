# Base image: Ubuntu 20.04 (required for ROS Noetic)
FROM ubuntu:20.04

# Set environment variables to avoid manual timezone prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary system dependencies and utilities
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    lsb-release \
    gnupg \
    build-essential \
    python3-pip \
    git \
    locales \
    && rm -rf /var/lib/apt/lists/*

# Set locale
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Add ROS Noetic repository and install ROS Noetic Desktop Full
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt-get update && apt-get install -y ros-noetic-ros-base 

# Add ROS environment to bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]  

# Install Miniconda (simple Conda)
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh && \
    bash /tmp/miniconda.sh -b -p /opt/miniconda && \
    rm /tmp/miniconda.sh && \
    /opt/miniconda/bin/conda init bash

# Add Conda to PATH
ENV PATH="/opt/miniconda/bin:$PATH"

# Verify ROS and Conda installation
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "export PATH=/opt/miniconda/bin:$PATH" >> ~/.bashrc

# Set entry point to bash
ENTRYPOINT ["/bin/bash"]
