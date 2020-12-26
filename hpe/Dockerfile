FROM pytorch/pytorch:1.7.0-cuda11.0-cudnn8-runtime 

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
    software-properties-common \ 
    python \
    python-pip \
    python3 \ 
    python3-pip \
    git \ 
    nano \
    sudo 

# Add developer user
RUN adduser --disabled-password \
    --gecos '' ${USER}
RUN adduser ${USER} sudo 
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers 

WORKDIR /home/${USER}
# Use conda to install neccessary packages 
RUN git clone https://github.com/Naman-ntc/Pytorch-Human-Pose-Estimation.git

RUN whereis conda
RUN conda install -c anaconda scipy configargparse cython
RUN conda install -c conda-forge progress json_tricks 

CMD ["bash"]
