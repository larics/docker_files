FROM pytorch/pytorch:1.1.0-cuda10.0-cudnn7.5-devel

LABEL maintainer filip.zoric@fer.hr

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
    python-pip \
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
RUN ./autogen.sh 
WORKDIR /protobuf 
RUN ./configure
RUN make \
    && make check \
    && make install \
    && ldconfig


RUN apt-get install -y openssl
# Build cmake
WORKDIR /
RUN git clone https://github.com/Kitware/CMake.git
RUN apt-get install -y libssl-dev
WORKDIR /CMake
RUN git checkout -b v3.12.0
RUN ./bootstrap \
    && make \
    && sudo make install 

# RUN 
RUN apt-get install -y --no-install-recommends \
    libhdf5-dev \
    libgoogle-glog-dev


# Add developer user / remove neccessity to type password for sudo command
RUN adduser --disabled-password \
    --gecos '' ${USER}
RUN adduser ${USER} sudo 
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers 

# Install packages neccessary for training and testing neural networks
RUN conda install -c anaconda \
                     scipy \
                     configargparse \
                     cython

RUN conda install -c conda-forge \
                     progress \
                     json_tricks \
                     tensorboardx \
                     easydict \
                     opencv \
                     pycocotools

RUN apt-get install -y libboost-dev 
# Clone OpenPose from source
WORKDIR /home/developer
RUN git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose
WORKDIR /home/developer/openpose
RUN git checkout tags/v1.7.0
RUN git submodule update --init --recursive --remote 

CMD ["bash"]
