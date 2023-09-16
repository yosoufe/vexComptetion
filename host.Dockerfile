FROM pytorch/pytorch:2.0.1-cuda11.7-cudnn8-devel

ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update \
  && apt-get upgrade -y \
  && apt-get install -y \
    git wget \
    libgl1-mesa-glx ffmpeg libsm6 libxext6 mesa-utils \
    kmod kbd nano

# Cortano
RUN git clone https://github.com/timrobot/Cortano.git
RUN cd Cortano \
  && python -m pip install . \
  && python -m pip install \
    open3d \
    cupoch \
    pyapriltags \
    jupyter \
    keyboard

# Download model
RUN wget --show-progress\
    -P /root/.cache/torch/hub/checkpoints/ \
    https://download.pytorch.org/models/maskrcnn_resnet50_fpn_coco-bf2d0c1e.pth


RUN sed -i "s/lan.write(self.motor_vals)/lan.write(self.motor)/g" /opt/conda/lib/python3.10/site-packages/cortano/interface.py 
