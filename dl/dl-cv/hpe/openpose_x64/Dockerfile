FROM nvidia/cuda:12.0.1-devel-ubuntu20.04
LABEL maintainer filip.zoric@fer.hr

# Env vars needed for NVIDIA container runtime (TODO: After build, pass them on the start of the Dockerfile) 
ENV LANG C.UTF-8
ENV LC_AL C.UTF-8
ENV DEBIAN_FRONTED noninteractive
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV TZ=Europe/Zagreb


RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo ${TZ} > /etc/timezone


# Set arguments 
ARG USER=developer 
ARG UID=1000
ARG GID=1000 
ARG PW=developer

# Install necessary software packages 
RUN apt-get update && \
    apt-get install -y --no-install-recommends \ 
    autoconf \
    automake \
    build-essential \
    feh \ 
    git \
    g++ \
    libtool \
    make \
    nano \
    python \
    python3 \
    python3-pip \
    software-properties-common \ 
    sudo \ 
    unzip \ 
    wget 



# Build protobuf
WORKDIR /
RUN git clone https://github.com/protocolbuffers/protobuf.git 
WORKDIR /protobuf
RUN git submodule update --init --recursive 
WORKDIR /protobuf 
RUN git checkout tags/v3.13.0
RUN ./autogen.sh
RUN ./configure
RUN make \
    && make install \
    && ldconfig

RUN apt-get install -y \
    openssl \
    qtbase5-dev
# Build cmake
WORKDIR /
RUN git clone https://github.com/Kitware/CMake.git
RUN apt-get install -y libssl-dev
WORKDIR /CMake
RUN git checkout tags/v3.20.0
RUN ./configure --qt-gui 
RUN ./bootstrap \
    && make \
    && sudo make install 

# Install some libs needed for openpose 
RUN apt-get install -y --no-install-recommends \
    libhdf5-dev \
    libgoogle-glog-dev


# Add developer user / remove neccessity to type password for sudo command
RUN adduser --disabled-password \
    --gecos '' ${USER}
RUN adduser ${USER} sudo 
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers 

# Install packages neccessary for training and testing neural networks
#RUN conda install -c anaconda \
#                     scipy \
#                     configargparse \
#                     cython

#RUN conda install -c conda-forge \
#                     progress \
#                     json_tricks \
#                     tensorboardx \
#                     easydict \
#                     opencv \
#                     pycocotools

ENV TZ=Europe/Zagreb 
RUN ln -snf /usr/share/zoneinfo/${TZ} /etc/localtime && echo ${TZ} > /etc/timezone 

RUN apt-get install -y \ 
    libboost-all-dev \
    libatlas-base-dev \ 
    libhdf5-dev \
    libgoogle-glog-dev \
    liblapacke-dev \
    libpthread-stubs0-dev \
    libopencv-dev \
    locate 

ENV ROS_DISTRO noetic
# ROS Installation 
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN sudo apt-get update \
    && sudo apt-get install -y \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport

# Manual boost download (hassle with manual version definition) 
#WORKDIR /home/developer
#RUN git clone --recursive https://github.com/boostorg/boost.git
#WORKDIR /home/developer/boost 
#RUN git fetch --all --tags
#RUN git checkout --force boost-1.65.1
#RUN mkdir build 
#RUN ./bootstrap.sh --prefix=./build 
#RUN ./b2 headers

WORKDIR /home/developer
# Provided prebuilt versions (allegedly/not sure if only headers)
#ARG BOOST_VERSION 
#ARG BOOST_VERSION_
#ENV BOOST_VERSION=${BOOST_VERSION}
#ENV BOOST_VERSION_=${BOOST_VERSION_}

#RUN wget --max-redirect 3 https://dl.bintray.com/boostorg/release/${BOOST_VERSION}/source/boost_${BOOST_VERSION_}.tar.gz
#RUN mkdir -p /usr/include/boost && tar zxf boost_${BOOST_VERSION_}.tar.gz -C /usr/include/boost --strip-components=1
#RUN echo ${BOOST_ROOT}

# Update g++/gcc for succesful openpose build 
RUN sudo add-apt-repository ppa:ubuntu-toolchain-r/test
RUN sudo apt update
RUN sudo apt install g++-7 -y
RUN sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 60 \
                         --slave /usr/bin/g++ g++ /usr/bin/g++-7 
RUN sudo update-alternatives --config gcc

# Clone OpenPose from source -> ros_openpose requires root
WORKDIR /root
RUN git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose
WORKDIR /root/openpose
#RUN git checkout tags/v1.7.0
RUN git submodule update --init --recursive --remote 
# There is possibility of automated openPose build 

## CMAKE-GUI
## https://github.com/CMU-Perceptual-Computing-Lab/openpose/issues/1953
## compute 86, for RTX graphics
## NO CUDNN

CMD ["bash"]
