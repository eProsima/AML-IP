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

The demo that is presented here follows the scheme of the figure below:

.. figure:: /rst/figures/rosbot2r_inference_demo.png

.. note::

    This tutorial assumes the reader has already reviewed :ref:`previous tutorial <demo_inference>`, understands how Edge and Inference Nodes work and what the installation requirements are.

Build the image
===============

Build `amlip-demos:inference-tensor-flow` Docker Image from the workspace where the `Dockerfile` is located.
In order to do so, execute the following:

.. code-block:: bash

    docker build -t amlip-demos:inference-tensor-flow -f Dockerfile .

.. note::

    Use --no-cache argument to restart build.

Environment
===========

The Docker Compose used for the simulation is ``compose.yaml``.
You can find it `here <https://github.com/eProsima/AML-IP/blob/main/amlip_demo_nodes/amlip_tensorflow_inference_demo/compose.yaml>`__.

The Docker Compose launches the following containers:

* **astra**:

.. code-block:: yaml

    astra:
        image: husarion/astra:humble
        devices:
        - /dev/bus/usb/
        volumes:
        - ./astra_params.yaml:/ros2_ws/install/astra_camera/share/astra_camera/params/astra_mini_params.yaml
        command: ros2 launch astra_camera astra_mini.launch.py

* **rosbot**:

.. code-block:: yaml

    rosbot:
        image: husarion/rosbot:humble
        command: ros2 launch rosbot_bringup bringup.launch.py mecanum:=False

* **microros**:

.. code-block:: yaml

    microros:
        image: husarion/micro-ros-agent:humble
        devices:
        - ${SERIAL_PORT:?err}
        environment:
        - SERIAL_PORT
        command: ros2 run micro_ros_agent micro_ros_agent serial -D $SERIAL_PORT serial -b 576000 # -v6

* **amlip-edge**:

.. code-block:: yaml

    amlip-edge:
        image: amlip-demos:inference-tensor-flow
        command:



Run demo
========

Bring up ROSbot2R and run Edge Node
-----------------------------------

First, it is necessary to launch the docker compose ``compose.yaml`` that will activate the containers ``astra``, ``rosbot`` and ``microros``.

Start the containers in a new `ROSbot` terminal, run the following command:

.. code-block:: bash

    cd ~/AML-IP-demos-ws/src/AML-IP/amlip_demo_nodes/amlip_tensorflow_inference_demo
    docker compose up

Run Inference Node
------------------

In the second terminal, run the following command in order to process the inference:

.. code-block:: bash

    # Source colcon installation
    source install/setup.bash

    # To execute Inference Node with pre-trained model from TensorFlow
    cd ~/AML-IP-demos-ws/src/AML-IP/amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo
    python3 inference_node_async.py
