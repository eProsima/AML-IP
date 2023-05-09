
#########################################################################################
# AML-IP Demos Dockerfile
#########################################################################################

FROM ubuntu:focal

# Avoids using interactions during building
ENV DEBIAN_FRONTEND=noninteractive

# Use a bash shell so it is possigle to run things like `source` (required for colcon builds)
SHELL ["/bin/bash", "-c"]

ARG amlip_branch="demo/algebraic"

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
        wget && \
    pip3 install -U colcon-common-extensions vcstool

# Install AML-IP
WORKDIR /AML-IP
RUN mkdir src && \
    wget https://raw.githubusercontent.com/eProsima/AML-IP/$git_branch/amlip.repos && \
    vcs import src < amlip.repos && \
    colcon build --packages-up-to amlip_demo_nodes agent_tool

# Source built workspace
RUN echo "source /AML-IP/install/setup.bash" >> ~/.bashrc
