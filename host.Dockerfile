FROM pytorch/pytorch:2.0.1-cuda11.7-cudnn8-devel

RUN apt-get update \
  && apt-get upgrade -y \
  && apt-get install -y git wget

# libgl
ENV DEBIAN_FRONTEND noninteractive
RUN apt install -y libgl1-mesa-glx ffmpeg libsm6 libxext6 mesa-utils

# Cortano
RUN git clone https://github.com/timrobot/Cortano.git
RUN cd Cortano \
  && python -m pip install . \
  && python -m pip install \
    open3d \
    cupoch \
    pyapriltags \
    jupyter

# Download model
RUN wget --show-progress\
    -P /root/.cache/torch/hub/checkpoints/ \
    https://download.pytorch.org/models/maskrcnn_resnet50_fpn_coco-bf2d0c1e.pth