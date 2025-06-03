# syntax=docker/dockerfile:1.3
FROM nvidia/opengl:1.2-glvnd-runtime-ubuntu22.04
LABEL maintainer=filip.zoric@fer.hr

# Environment setup
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute

# Set timezone to avoid tzdata interactive prompt
ENV TZ=Etc/UTC
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    cmake \
    ninja-build \
    curl \
    wget \
    libx11-dev \
    libxi-dev \
    libxxf86vm-dev \
    libxcursor-dev \
    libxrandr-dev \
    libxinerama-dev \
    libglew-dev \
    libfreetype6-dev \
    libglu1-mesa-dev \
    libssl-dev \
    libtbb-dev \
    libopenal-dev \
    libpulse-dev \
    libsdl2-dev \
    libepoxy-dev \
    libjack-jackd2-dev \
    libavdevice-dev \
    libavformat-dev \
    libavutil-dev \
    libavcodec-dev \
    libswscale-dev \
    libx264-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    python3 \
    python3-dev \
    python3-pip \
    libwayland-dev \
    libxkbcommon-dev \
    libegl1-mesa-dev \
    xz-utils \
    nano \
    sudo \
    ca-certificates \
    software-properties-common \
    subversion \
    liblzma-dev

# Install newer GCC (Blender 4.4 recommends GCC >= 9)
RUN add-apt-repository ppa:ubuntu-toolchain-r/test -y && \
    apt-get update && \
    apt-get install -y gcc-12 g++-12 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 100 --slave /usr/bin/g++ g++ /usr/bin/g++-12

# Clone Blender source
RUN mkdir /home/blender-git
WORKDIR /home/blender-git
RUN git clone https://projects.blender.org/blender/blender.git
WORKDIR /home/blender-git/blender
RUN git checkout tags/v4.4.0

# Get precompiled libraries
WORKDIR /home/blender-git
RUN mkdir lib && cd lib && \
    wget https://builder.blender.org/download/daily/lib/linux_x86_64_glibc_228.zip && \
    unzip linux_x86_64_glibc_228.zip && \
    rm linux_x86_64_glibc_228.zip

# Configure environment for Blender build
ENV LIBDIR=/home/blender-git/lib/linux_x86_64_glibc_228
ENV CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Release -DWITH_CYCLES_DEVICE_CUDA=ON -DWITH_CYCLES_DEVICE_OPTIX=ON -DWITH_INSTALL_PORTABLE=ON"

# Make and build Blender
WORKDIR /home/blender-git/blender
RUN make update
RUN make -j$(nproc)

CMD ["/bin/bash"]
