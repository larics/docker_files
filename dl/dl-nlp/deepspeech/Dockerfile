FROM nvidia/cuda:10.1-cudnn7-devel-ubuntu18.04

LABEL maintainer filip.zoric@fer.hr

# Mic not working in docker :S 
RUN apt-get update \
    && apt-get install -y \
    apt-utils \
    build-essential \
    bc \
    cmake \
    curl \
    git \
    imagemagick \
    lsb-release 


# Add pulseaudio stuff and corresponding entrypoint
RUN apt-get install -y --no-install-recommends \
    alsa-utils \
    pulseaudio-utils \
    pulseaudio \
    portaudio19-dev \
    libpulse0 \
    libasound-dev \
    swig \
    python-pip \
    sudo

#COPY entrypoint.sh /opt/bin/entrypoint.sh
RUN adduser root pulse-access
# Add user and add it to sudo group
RUN adduser --disabled-password --gecos '' developer 
RUN adduser developer sudo 
RUN usermod -a -G audio developer
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER developer
WORKDIR /home/developer

# Install python3 packeges
RUN mkdir tools
WORKDIR /home/developer/tools
COPY pa_stable_candidate_v190700_rc2_20210331.tgz .
RUN tar zxvf pa_stable_candidate_v190700_rc2_20210331.tgz
WORKDIR /home/developer/tools/portaudio
RUN ./configure 
RUN make 
RUN sudo make install
RUN sudo ldconfig

RUN sudo apt-get install -y \
    python-dev \
    python-setuptools \
    python3-dev \
    python3-pip
RUN pip3 install pyaudio 

# Copy pulse-client (enable server/client arch)
COPY pulse-client.conf /etc/pulse/client.conf

# Install deepspeech

WORKDIR /home/developer
RUN pip3 install \
    deepspeech \
    deepspeech-gpu \
    halo \
    scipy \
    numpy \
    wave \
    pyaudio \
    webrtcvad

RUN mkdir deepspeech
WORKDIR /home/developer/deepspeech
# Download pre-trained English model files
RUN curl -LO https://github.com/mozilla/DeepSpeech/releases/download/v0.9.3/deepspeech-0.9.3-models.pbmm
RUN curl -LO https://github.com/mozilla/DeepSpeech/releases/download/v0.9.3/deepspeech-0.9.3-models.scorer


WORKDIR /home/developer
# Clone deepspeech examples
RUN git clone https://github.com/mozilla/DeepSpeech-examples

# Setup root user (neccessary for audio setup) 
USER root

# Copy entrypoint script
COPY entrypoint.sh /root/entrypoint.sh 

# Run entrypoint script
ENTRYPOINT /root/entrypoint.sh
