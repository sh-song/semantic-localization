FROM ros:noetic-ros-base-focal
ARG DEBIAN_FRONTEND=noninteractive

#Fundamentals
RUN \
    apt update -y && \
    apt upgrade -y && \
    apt install -y vim git

#HPP Environment
RUN apt install -y \
        g++ \
        python3-pip

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


## For parsing NGII map
RUN pip3 install \
        utm \
        pymap3d \
        tqdm \
        geopandas==0.8.2

## For ROS
#ROS
RUN apt install -y \
	build-essential \
	cmake \
	libyaml-cpp-dev \
	libpcap-dev \
	libeigen3-dev \
	libjsoncpp-dev

#ROS python
RUN \
	apt install -y python3-pip && \
	apt install -y python3-catkin-tools python3-vcstool python3-osrf-pycommon

# ROS msg
RUN apt install -y ros-noetic-novatel-oem7-driver

#Directory
RUN mkdir -p /workspace/output
WORKDIR /workspace

#For displaying matplotlib 3d
RUN apt install -y python3-tk

#
RUN pip3 install pyrender
RUN pip3 install lmfit
