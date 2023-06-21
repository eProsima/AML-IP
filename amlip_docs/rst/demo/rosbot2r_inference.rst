.. include:: ../exports/alias.include

.. _demo_rosbot2r_inference:

########################################
TensorFlow Inference using ROSbot2R Demo
########################################

.. contents::
    :local:
    :backlinks: none
    :depth: 2

Background
==========

This document provides detailed instructions on deploying an Edge Node on a *ROSbot 2R* from Husarion.
The Edge Node will capture an image using the Orbbec Astra camera and sends it to the Inference Node deployed on a laptop to perform TensorFlow inference on the given image.

In this demo we will use the simulation of `ROSbot 2R` - an autonomous mobile robot by `Husarion <https://husarion.com/manuals/rosbot/>`_, designed for learning ROS and for research and development purposes.
It is an affordable platform that can serve as a base for a variety of robotic applications, including inspection robots and custom service robots.
The robot features a solid aluminum frame and is equipped with a Raspberry Pi 4 with 4GB of RAM, distance sensors, an RPLIDAR A2 laser scanner, and an RGB-D Orbbec Astra camera.

.. figure:: /rst/figures/rosbot2r.png
    :align: center
    :width: 50%

The demo that is presented here follows the scheme of the figure below:

.. figure:: /rst/figures/rosbot2r_inference_demo.png

.. note::

    This tutorial assumes the reader has already reviewed :ref:`previous tutorial <demo_inference>`, understands how Edge and Inference Nodes work and what the installation requirements are.

Prerequisites
=============

Build `amlip-demos:inference-tensor-flow` Docker Image from the workspace where the `Dockerfile` is located.
In order to do so, execute the following:

.. code-block:: bash

    cd ~/AML-IP-ws/src/AML-IP/amlip_demo_nodes/amlip_tensorflow_inference_rosbot2r_demo
    docker build -t amlip-demos:inference-tensor-flow -f Dockerfile .

.. note::

    Use --no-cache argument to restart build.

Environment
===========

The Docker Compose used for the simulation is ``compose.yaml``.
You can find it `here <https://github.com/eProsima/AML-IP/blob/main/amlip_demo_nodes/amlip_tensorflow_inference_rosbot2r_demo/compose.yaml>`__.

The Docker Compose launches the following containers:

* **astra**: allows the usage of Orbbec 3D cameras with ROS Humble.

.. code-block:: yaml

    astra:
        image: husarion/astra:humble
        network_mode: host
        ipc: host
        devices:
        - /dev/bus/usb/
        volumes:
        - ./astra_params.yaml:/ros2_ws/install/astra_camera/share/astra_camera/params/astra_mini_params.yaml
        privileged: true
        command: ros2 launch astra_camera astra_mini.launch.py

* **rosbot**: starts all base functionalities for ROSbot 2R.

.. code-block:: yaml

    rosbot:
        image: husarion/rosbot:humble
        network_mode: host
        ipc: host
        privileged: true
        command: ros2 launch rosbot_bringup bringup.launch.py mecanum:=False

* **microros**: communicates with all firmware functionalities.

.. code-block:: yaml

    microros:
        image: husarion/micro-ros-agent:humble
        network_mode: host
        ipc: host
        devices:
        - ${SERIAL_PORT:?err}
        environment:
        - SERIAL_PORT
        privileged: true
        command: ros2 run micro_ros_agent micro_ros_agent serial -D $SERIAL_PORT serial -b 576000 # -v6

* **edge**: is responsible for starting up the execution of the Edge Node.

.. code-block:: yaml

    edge:
        image: amlip-demos:inference-tensor-flow
        network_mode: host
        ipc: host
        privileged: true
        command: bash -c "sleep 5 && source ./install/setup.bash && python3 ./src/amlip/amlip_demo_nodes/amlip_tensorflow_inference_rosbot2r_demo/amlip_tensorflow_inference_rosbot2r_demo/edge_node_async.py"



Run demo
========

Bring up ROSbot2R and run Edge Node
-----------------------------------

First, it is necessary to launch the docker compose ``compose.yaml`` that will activate the containers ``astra``, ``rosbot``, ``microros`` and ``edge``.

Start the containers in a new `ROSbot` terminal, run the following command:

.. code-block:: bash

    cd ~/AML-IP-ws/src/AML-IP/amlip_demo_nodes/amlip_tensorflow_inference_rosbot2r_demo
    docker compose up

Run Inference Node
------------------

In a terminal on your laptop, run the following command in order to process the inference:

.. code-block:: bash

    # Source colcon installation
    source install/setup.bash

    # To execute Inference Node with pre-trained model from TensorFlow
    cd ~/AML-IP-ws/src/AML-IP/amlip_demo_nodes/amlip_tensorflow_inference_rosbot2r_demo/amlip_tensorflow_inference_rosbot2r_demo
    python3 inference_node_async.py
