FROM ros_noetic_conda:latest 

ENV DEBIAN_FRONTEND=noninteractive

RUN conda create -y -n vit_pose python=3.7


SHELL ["conda", "run", "-n", "vit_pose", "/bin/bash", "-c"] 
WORKDIR /root
RUN git clone https://github.com/open-mmlab/mmcv.git

# Install mmcv 
WORKDIR /root/mmcv
RUN git checkout v1.3.9 && \ 
    MMCV_WITH_OPS=1 pip install -e .

WORKDIR /root/
RUN git clone https://github.com/ViTAE-Transformer/ViTPose.git
WORKDIR /root/ViTPose
RUN  pip install -v -e . 

# Don't know why is this expose here? 
#EXPOSE 5003
#ENTRYPOINT ["conda", "activate", "vit_pose"] 
CMD ["bash"]  
