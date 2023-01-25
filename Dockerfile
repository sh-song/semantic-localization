FROM ubuntu:20.04
ARG DEBIAN_FRONTEND=noninteractive

#Fundamentals
RUN \
    apt update -y && \
    apt upgrade -y && \
    apt install -y vim git

#HPP Environment
RUN apt install -y \
        g++ \
        python3-pip \
        libblas-dev \
        liblapack-dev \
        libpng-dev \
        libfreetype6-dev \
        linux-tools-5.15.0-48-generic

#HPP Python
RUN pip3 install \
        numpy \
        scipy \
        matplotlib \
        snakeviz \
        line_profiler \
        psutil \
        memory_profiler \
        py-spy \
        Cython

#CV
RUN apt install -y \
    libopencv-dev

RUN pip3 install opencv-python==4.6.0.66
#Directory
RUN mkdir -p /workspace/output
WORKDIR /workspace

#For displaying matplotlib 3d
RUN apt install -y python3-tk
