FROM pytorch/pytorch:2.0.1-cuda11.7-cudnn8-devel

RUN apt-get update \
  && apt-get upgrade -y \
  && apt-get install -y git

# libgl
ENV DEBIAN_FRONTEND noninteractive
RUN apt install -y libgl1-mesa-glx ffmpeg libsm6 libxext6 mesa-utils

# Cortano
RUN git clone https://github.com/timrobot/Cortano.git
RUN cd Cortano \
  && python -m pip install .
