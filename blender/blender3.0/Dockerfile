# syntax=docker/dockerfile:1.3
FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu18.04
LABEL maintainer filip.zoric@fer.hr 

# Setup environment 
ENV LANG C.UTF-8
ENV LC_AL C.UTF-8
ENV DEBIAN_FRONTEND noninteractive
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# Setup timezone (fix interactive package installation) 
RUN ln -snf /usr/share/zoneinfo*${TZ} /etc/localtime && echo ${TZ} >/etc/timezone

RUN apt-get update && \
    apt-get install -y \
    curl \
    git \
    libfreetype6 \
    libglu1-mesa \
    libxi6 \
    libxrender1 \
    nano \
    sudo \
    xz-utils \
    wget 

RUN apt-get update && \
    apt-get install -y \
    subversion \
    cmake \
    libx11-dev \
    libxxf86vm-dev \
    libxcursor-dev \
    libxi-dev \
    libxrandr-dev \
    libxinerama-dev 


RUN apt-get install --reinstall ca-certificates
RUN mkdir /home/blender-git
WORKDIR /home/blender-git 
RUN git clone https://git.blender.org/blender.git
WORKDIR /home/blender-git/blender
RUN git checkout tags/v2.80

WORKDIR /home/blender-git/lib
RUN svn checkout https://svn.blender.org/svnroot/bf-blender/trunk/lib/linux_centos7_x86_64

RUN apt-get install -y software-properties-common
RUN add-apt-repository ppa:ubuntu-toolchain-r/test
RUN apt update
RUN apt install -y gcc-9
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 90 --slave /usr/bin/g++ g++ /usr/bin/g++-9 --slave /usr/bin/gcov gcov /usr/bin/gcov-9
RUN apt-get install -y \
    python3 \
    build-essential

WORKDIR /home/blender-git/blender
RUN make update 
RUN make -j7


CMD ["/bin/bash"]
