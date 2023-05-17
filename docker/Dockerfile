
#########################################################################################
# AML-IP Demos Dockerfile
#########################################################################################

FROM ubuntu:jammy

# Avoids using interactions during building
ENV DEBIAN_FRONTEND=noninteractive

# Use a bash shell so it is possigle to run things like `source` (required for colcon builds)
SHELL ["/bin/bash", "-c"]

ARG amlip_branch="demo/algebraic"

# Avoid interactuation with installation of some package that needs the locale.
ENV TZ=Europe/Madrid
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Update Python to 3.11
RUN apt update && \
    apt install -y software-properties-common && \
    apt update && \
    add-apt-repository ppa:deadsnakes/ppa && \
    apt update && \
    apt install -y python3.11 && \
    update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1 && \
    update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.11 2 && \
    update-alternatives --config python3 && \
    python3 -v

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
    wget https://raw.githubusercontent.com/eProsima/AML-IP/$amlip_branch/amlip.repos && \
    vcs import src < amlip.repos && \
    colcon build --packages-up-to-regex amlip

# Source built workspace
RUN echo "source /AML-IP/install/setup.bash" >> ~/.bashrc
