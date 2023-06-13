
#########################################################################################
# AML-IP Demos Dockerfile
#########################################################################################

FROM ubuntu:22.04

# Avoids using interactions during building
ENV DEBIAN_FRONTEND=noninteractive

# Use a bash shell so it is possigle to run things like `source` (required for colcon builds)
SHELL ["/bin/bash", "-c"]

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
        espeak \
        alsa-utils \
        libopencv-dev \
        ffmpeg && \
    pip3 install -U colcon-common-extensions \
        vcstool \
        pyttsx3 \
        opencv-python \
        tensorflow \
        tensorflow-hub \
        tensorflow-object-detection-api \
        pygame

# Install AML-IP
WORKDIR /AML-IP
RUN mkdir src && \
    git clone -b master https://github.com/eProsima/foonathan_memory_vendor.git /AML-IP/src/foonathan_memory_vendor && \
    git clone -b master https://github.com/eProsima/Fast-CDR.git /AML-IP/src/Fast-CDR && \
    git clone -b master https://github.com/eProsima/Fast-DDS.git /AML-IP/src/Fast-DDS && \
    git clone -b main https://github.com/eProsima/dev-utils.git /AML-IP/src/dev-utils && \
    git clone -b main https://github.com/eProsima/DDS-Router.git /AML-IP/src/DDS-Router && \
    git clone -b feature/edge_inference https://github.com/eProsima/AML-IP.git /AML-IP/src/AML-IP && \
    colcon build

# Source built workspace
RUN echo "source /AML-IP/install/setup.bash" >> ~/.bashrc && \
    echo "source /AML-IP-demos/install/setup.bash" >> ~/.bashrc
