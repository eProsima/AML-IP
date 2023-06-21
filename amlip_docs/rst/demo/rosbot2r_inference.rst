.. include:: ../exports/alias.include

.. _demo_rosbot2r_inference:

####################################
TensorFlow Inference using ROSbot 2R
####################################

.. contents::
    :local:
    :backlinks: none
    :depth: 2

Background
==========

This document provides detailed instructions on deploying an Edge Node on a *ROSbot 2R* from Husarion.
The Edge Node will capture images using the Orbbec Astra camera and sends them to the Inference Node deployed on a laptop to perform TensorFlow inference on the given image.
If the TensorFlow inference detects the presence of a person with a probability of 80% or higher, the robot will turn.

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

Build `amlip-demos:inference-tensorflow` Docker Image from the workspace where the `Dockerfile` is located.
In order to do so, execute the following:

.. code-block:: bash

    cd ~/AML-IP-ws/src/AML-IP/amlip_demo_nodes/amlip_tensorflow_inference_rosbot2r_demo
    docker build -t amlip-demos:inference-tensorflow -f Dockerfile .

.. note::

    Use --no-cache argument to restart build.

ROSbot 2R Deployment
====================

The Docker Compose used for the demo is ``compose.yaml``.
You can find it `here <https://github.com/eProsima/AML-IP/blob/main/amlip_demo_nodes/amlip_tensorflow_inference_rosbot2r_demo/compose.yaml>`__.

The Docker Compose launches the following containers:

* **astra**: allows the usage of Orbbec 3D cameras with ROS Humble. It publishes the images captured by the camera to the ``/camera/color/image_raw`` topic. Edge Node can then subscribe to this topic to receive and process the camera images.

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

* **rosbot**: starts all base functionalities for ROSbot 2R. It subscribes to the ``/cmd_vel`` topic. This topic is used to control the movement of a robot by publishing velocity commands to it.

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

* **edge**: is responsible for starting up the execution of the AML-IP Edge Node explained below.

.. code-block:: yaml

    edge:
        image: amlip-demos:inference-tensorflow
        network_mode: host
        ipc: host
        privileged: true
        command: bash -c "sleep 5 && source ./install/setup.bash && python3 ./src/amlip/amlip_demo_nodes/amlip_tensorflow_inference_rosbot2r_demo/amlip_tensorflow_inference_rosbot2r_demo/edge_node_async.py"

The following diagram illustrates the flow of the explained code:

.. figure:: /rst/figures/rosbot2r_inference_details.png

Working with AML-IP
===================

Through this section, we will delve into the details of the demo, examining the underlying concepts and processes involved.

Edge Node
---------

Edge Node serves as the entity responsible for sending the data to be inferred to the Inference Node.
The Edge Node is typically located at the edge of a network or closer to the data source, such as a sensor or a device generating the data.
In this specific scenario, the data source is the camera of the robot.

The Python code for the Edge Node is explained in the :ref:`previous tutorial <demo_inference>`, so here we will focus on the additional features added to this demo.
You can find the complete code `here <https://github.com/eProsima/AML-IP/blob/main/amlip_demo_nodes/amlip_tensorflow_inference_rosbot2r_demo/amlip_tensorflow_inference_rosbot2r_demo/edge_node_async.py>`__.

The next block includes the Python header files that allow the use of the AML-IP Python API and ROS 2.

.. literalinclude:: /../amlip_demo_nodes/amlip_tensorflow_inference_rosbot2r_demo/amlip_tensorflow_inference_rosbot2r_demo/edge_node_async.py
  :language: python
  :lines: 19-35

Continuing, the ``SubscriberImage`` ROS 2 Node subscribes to the ``/camera/color/image_raw`` topic to receive images from a camera sensor.
It converts the received ROS Image message to an OpenCV image using the ``CvBridge`` package.
The image data is stored in the image attribute of the node.

.. literalinclude:: /../amlip_demo_nodes/amlip_tensorflow_inference_rosbot2r_demo/amlip_tensorflow_inference_rosbot2r_demo/edge_node_async.py
  :language: python
  :lines: 45-67

The ``PublisherVel`` ROS 2 Node publishes ``Twist`` messages to the ``/cmd_vel`` topic, which controls the velocity of the ROSbot 2R.
In the provided code, the ``turn`` method is implemented to set linear and angular velocities, causing the robot to turn.

.. literalinclude:: /../amlip_demo_nodes/amlip_tensorflow_inference_rosbot2r_demo/amlip_tensorflow_inference_rosbot2r_demo/edge_node_async.py
  :language: python
  :lines: 70-95

Then, the definition of ``turn_rosbot`` function initializes the ``PublisherVel`` Node and repeatedly calls the ``turn`` method to make the ROSbot turn.

.. literalinclude:: /../amlip_demo_nodes/amlip_tensorflow_inference_rosbot2r_demo/amlip_tensorflow_inference_rosbot2r_demo/edge_node_async.py
  :language: python
  :lines: 98-108

After that, the ``check_data`` function extracts labels and percentages from the inference string received from the Edge Node.
It searches for the label ``person`` with a confidence percentage greater than or equal to ``80%``.
If a person is detected, it calls the ``turn_rosbot`` function to make the robot turn.

.. literalinclude:: /../amlip_demo_nodes/amlip_tensorflow_inference_rosbot2r_demo/amlip_tensorflow_inference_rosbot2r_demo/edge_node_async.py
  :language: python
  :lines: 111-123

The ``main`` function initializes the ``SubscriberImage`` Node to receive images from the ROSbot 2R camera and waits until an image arrives before proceeding.
The received image is then used to create the ``AsyncEdgeNode``.
The image is encoded as bytes and sent to the Edge Node for inference using the ``request_inference`` method as in the previous demo.

.. literalinclude:: /../amlip_demo_nodes/amlip_tensorflow_inference_rosbot2r_demo/amlip_tensorflow_inference_rosbot2r_demo/edge_node_async.py
  :language: python
  :lines: 136-182

Inference Node
--------------

The Inference Node is responsible for making the inferences or predictions on the data it receives using a TensorFlow model.
The Inference Node is typically a server or a computing resource equipped with high-performance hardware optimized for executing machine learning models efficiently.

The Python code for the Inference Node is explained in the previous tutorial and can be found `here <https://github.com/eProsima/AML-IP/blob/main/amlip_demo_nodes/amlip_tensorflow_inference_rosbot2r_demo/amlip_tensorflow_inference_rosbot2r_demo/inference_node_async.py>`__.

Run demo
========

Bring up ROSbot 2R and run Edge Node
------------------------------------

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

Teleoperate ROSbot 2R
---------------------

Note that ROSbot 2R subscribes to the ``/cmd_vel`` topic.
To teleoperate it, you can use the ``teleop_twist_keyboard`` node, which allows you to control the robot using keyboard inputs.
Follow these steps:

.. code-block:: bash

    # Source ROS 2 installation
    source install/setup.bash

    ros2 run teleop_twist_keyboard teleop_twist_keyboard
