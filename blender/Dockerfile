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
#    apt-get -y autoremove && \
#    rm -rf /var/lib/apt/lists/*

ENV BLENDER_MAJOR 2.79
ENV BLENDER_VERSION 2.79
ENV BLENDER_URL https://download.blender.org/release/Blender${BLENDER_MAJOR}/blender-${BLENDER_VERSION}-linux-glibc219-x86_64.tar.bz2

# https://github.com/luvres/blender 
# Index of blender releases
#ENV BLENDER_URL https://download.blender.org/release/Blender${BLENDER_MAJOR}/blender-${BLENDER_VERSION}-linux64.tar.xz

WORKDIR /home
RUN wget ${BLENDER_URL} 
RUN tar xvf blender-${BLENDER_VERSION}-linux-glibc219-x86_64.tar.bz2 
RUN mv blender-${BLENDER_VERSION}-linux-glibc219-x86_64 blender && \
    rm blender-${BLENDER_VERSION}-linux-glibc219-x86_64.tar.bz2

#RUN curl -L ${BLENDER_URL} | tar -xJ -C /usr/local/ && \
#    mv /usr/local/blender-${BLENDER_VERSION}-linux64 /usr/local/blender

#RUN curl -L ${BLENDER_URL} | tar jxf - -C /opt
#RUN ls -la
#    tar -xf blender*
#| tar -xJ -C /usr/local/ && \
#	mv /usr/local/blender-${BLENDER_VERSION}-linux64 /usr/local/blender

#VOLUME /media
#ENTRYPOINT ["/usr/local/blender/blender", "-b"]
CMD ["/bin/bash"]
