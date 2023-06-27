#########################################################################################
# AML-IP TensorFlow Inference using ROSbot2R Demo Dockerfile
#########################################################################################

FROM ros:humble-ros-core

# Avoids using interactions during building
ENV DEBIAN_FRONTEND=noninteractive

# Use a bash shell so it is possigle to run things like `source` (required for colcon builds)
SHELL ["/bin/bash", "-c"]

ARG amlip_branch="main"

# Avoid interactuation with installation of some package that needs the locale.
ENV TZ=Europe/Madrid
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Download system dependencies
RUN apt update && \
    apt install -y \
        cmake \
        g++ \
        git \
        libasio-dev \
        libssl-dev \
        libtinyxml2-dev \
        libyaml-cpp-dev \
        pip \
        swig \
        wget \
        alsa-utils \
        libopencv-dev \
        ros-humble-cv-bridge && \
    pip3 install -U colcon-common-extensions \
        vcstool \
        tensorflow \
        tensorflow-hub \
        tensorflow-object-detection-api

# Install AML-IP
WORKDIR /AML-IP
RUN mkdir src && \
    git clone -b master https://github.com/eProsima/foonathan_memory_vendor.git /AML-IP/src/foonathan_memory_vendor && \
    git clone -b master https://github.com/eProsima/Fast-CDR.git /AML-IP/src/fastcdr && \
    git clone -b master https://github.com/eProsima/Fast-DDS.git /AML-IP/src/fastdds && \
    git clone -b main https://github.com/eProsima/dev-utils.git /AML-IP/src/dev-utils && \
    git clone -b main https://github.com/eProsima/DDS-Pipe.git /AML-IP/src/ddspipe && \
    git clone -b main https://github.com/eProsima/DDS-Router.git /AML-IP/src/ddsrouter && \
    git clone -b feature/rosbot2r_inference https://github.com/eProsima/AML-IP.git /AML-IP/src/amlip && \
    colcon build --packages-up-to-regex amlip

# Source built workspace
RUN echo "source /AML-IP/install/setup.bash" >> ~/.bashrc && \
    echo "source /AML-IP/install/setup.bash" >> ~/.bashrc
