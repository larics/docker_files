FROM pytorch/pytorch:0.4.1-cuda9-cudnn7-devel

LABEL maintainer filip.zoric@fer.hr & denis.djurasinovic@gmail.com

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
RUN sudo apt install --only-upgrade python3


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
#
#RUN conda install -c conda-forge \
#                     progress \
#                     json_tricks \
#                     tensorboardx \
#                     easydict \
#                     opencv \
#                     pycocotools

# Clone human pose estimation from microsoft
WORKDIR /home/${USER}
RUN git clone https://github.com/microsoft/human-pose-estimation.pytorch.git

# Create input data directory
RUN mkdir data

# Add pycharm for faster development and testing 
ENV version 2020.2.3
#RUN wget -O /home/developer/pycharm.tar.gz "https://download.jetbrains.com/python/pycharm-community-$version.tar.gz"
#RUN tar -xvf /home/developer/pycharm.tar.gz && rm -rf /home/developer/pycharm.tar.gz 
#RUN echo 'alias pycharm=/home/developer/pycharm-community-$version/bin/pycharm.sh' >> /home/developer/.bashrc 

#Install dependencies, fix broken ones from requirements.txt
RUN pip install -r human-pose-estimation.pytorch/requirements.txt
RUN pip install numpy==1.16
RUN pip install matplotlib==3.0.3
RUN apt install -y libsm6
RUN apt install -y libxrender1
#RUN conda install -c pytorch pytorch cuda100	#For newer GPUs

#Make libs
RUN make -C human-pose-estimation.pytorch/lib/ 

#Install COCOAPI
RUN git clone https://github.com/cocodataset/cocoapi.git
RUN pip install cython
RUN make install -C cocoapi/PythonAPI

#Disable cudnn for batch_norm (github instructions)
RUN sed -i "1254s/torch\.backends\.cudnn\.enabled/False/g" /opt/conda/lib/python3.6/site-packages/torch/nn/functional.py

CMD ["bash"]
