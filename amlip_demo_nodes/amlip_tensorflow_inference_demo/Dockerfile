#########################################################################################
# AML-IP Demos Dockerfile
#########################################################################################

FROM ubuntu:jammy

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
    wget https://raw.githubusercontent.com/eProsima/AML-IP/$amlip_branch/amlip.repos && \
    vcs import src < amlip.repos && \
    colcon build --packages-up-to-regex amlip

# Source built workspace
RUN echo "source /AML-IP/install/setup.bash" >> ~/.bashrc && \
    echo "source /AML-IP-demos/install/setup.bash" >> ~/.bashrc
