# Use an appropriate CUDA base image
FROM nvidia/cuda:11.6.2-cudnn8-devel-ubuntu20.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=/opt/conda/bin:$PATH

# Update and install necessary packages, clean up after installation
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    bzip2 \
    git \
    ffmpeg \
    build-essential \
    ninja-build \
    cuda-libraries-11-6 \
    mesa-common-dev \
    libosmesa6 libosmesa6-dev \
    libgles2-mesa-dev \
    libglu1-mesa-dev \
    libcublas-11-6 \
    libcublas-dev-11-6 \
    libcusparse-dev-11-6 \
    libcusolver-dev-11-6 \
    libcurand-dev-11-6 \
    libcufft-dev-11-6 \
    nvidia-cuda-toolkit \
    libyaml-dev \
    cuda-nvcc-11-6 \
    cuda-nvrtc-dev-11-6 \
    cuda-nvml-dev-11-6 \
    cuda-toolkit-11-6 \
    python3-tk \
    bc \
    openssh-client && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Install Miniconda
RUN wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh && \
    bash /tmp/miniconda.sh -b -p /opt/conda && \
    rm /tmp/miniconda.sh&& \
    /opt/conda/bin/conda init bash

# Set up conda environment
ENV PATH=/opt/conda/bin:$PATH

SHELL ["/bin/bash", "-c"]

# Create conda environment and install packages
RUN conda create -n alphapose python=3.7 -y && \
    echo "conda activate alphapose" >> ~/.bashrc

SHELL ["conda", "run", "-n", "alphapose", "/bin/bash", "-c"]

RUN conda install -n alphapose -c pytorch pytorch==1.12.1 torchvision==0.13.1 torchaudio==0.12.1 cudatoolkit=11.6 -y -c conda-forge && \
    conda clean -afy

# Install additional Python packages
RUN /opt/conda/envs/alphapose/bin/pip install cython==0.29.33 
RUN /opt/conda/envs/alphapose/bin/pip install opencv-contrib-python-headless cython pycocotools cython_bbox==0.1.3 numpy==1.21.6 six terminaltables scipy matplotlib visdom tqdm tensorboardx easydict pyyaml halpecocotools munkres timm==0.1.20 natsort boto3 jupyterlab

# Verify CUDA installation
RUN /opt/conda/envs/alphapose/bin/python -c "import torch; print(torch.version.cuda)"

# Clone necessary repositories and build
RUN mkdir /build && \
    cd /build && \
    # git clone https://github.com/HaoyiZhu/HalpeCOCOAPI.git && \
    git clone https://github.com/MVIG-SJTU/AlphaPose.git && \
    # cd /build/HalpeCOCOAPI/PythonAPI && \
    # /opt/conda/envs/alphapose/bin/python setup.py build develop --user && \
    cd /build/AlphaPose && \
    TORCH_CUDA_ARCH_LIST="6.1;7.5;8.6" /opt/conda/envs/alphapose/bin/python setup.py build develop --user && \
    rm -rf /build/AlphaPose/build

RUN cd /build && \
    git clone https://github.com/HaoyiZhu/HalpeCOCOAPI.git && \
    cd /build/HalpeCOCOAPI/PythonAPI && \
    /opt/conda/envs/alphapose/bin/python setup.py build develop --user
    
WORKDIR /build/AlphaPose

# Pre-download ResNet models
RUN /opt/conda/envs/alphapose/bin/python -c "import torchvision.models as tm; tm.resnet152(pretrained=True)" && \
    /opt/conda/envs/alphapose/bin/python -c "import torchvision.models as tm; tm.resnet50(pretrained=True)"

# Set Python3 as the default Python version
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 2

# Clean up
RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* /root/.cache

#RUN mkdir -p detector/yolo/data
#COPY ./src/yolov3-spp.weights detector/yolo/data/
#RUN mkdir -p trackers/weights
#COPY ["./src/osnet_ain_x1_0_msmt17_256x128_amsgrad_ep50_lr0.0015_coslr_b64_fb10_softmax_labsmth_flip_jitter.pth", "trackers/weights/"]
#COPY ./src/singlehand_fast50_dcn_regression_256x192.pth pretrained_models/
#COPY ./src/multi_domain_fast50_dcn_combined_256x192.pth pretrained_models/
#COPY ./src/fast_res50_256x192.pth pretrained_models/


# test : 
# python scripts/demo_inference.py --cfg configs/coco/resnet/256x192_res50_lr1e-3_1x.yaml --checkpoint pretrained_models/fast_res50_256x192.pth --indir examples/demo/ --gpus -1 --debug --pose_track --outdir test
