FROM pytorch/pytorch:1.7.0-cuda11.0-cudnn8-devel

LABEL maintainer filip.zoric@fer.hr

# Set arguments 
ARG USER=developer 
ARG UID=1000
ARG GID=1000 
ARG PW=developer

# Install necessary software packages 
RUN apt-get update && \
    apt-get install -y --no-install-recommends \ 
    build-essential \
    feh \ 
    git \
    nano \
    python \
    python-pip \
    python3 \
    python3-pip \
    software-properties-common \ 
    sudo \ 
    wget 

# Add developer user / remove neccessity to type password for sudo command
RUN adduser --disabled-password \
    --gecos '' ${USER}
RUN adduser ${USER} sudo 
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers 

WORKDIR /home/${USER}
# Use conda to install neccessary packages 
RUN git clone https://github.com/Naman-ntc/Pytorch-Human-Pose-Estimation.git

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

# Clone human pose estimation from microsoft 
WORKDIR /home/${USER}
RUN git clone https://github.com/larics/hpe-pytorch.git

# Create input data directory
RUN mkdir data

# Add pycharm for faster development and testing 
ENV version 2020.2.3
RUN wget -O /home/developer/pycharm.tar.gz "https://download.jetbrains.com/python/pycharm-community-$version.tar.gz"
RUN tar -xvf /home/developer/pycharm.tar.gz && rm -rf /home/developer/pycharm.tar.gz 
RUN echo 'alias pycharm=/home/developer/pycharm-community-$version/bin/pycharm.sh' >> /home/developer/.bashrc 

CMD ["bash"]
