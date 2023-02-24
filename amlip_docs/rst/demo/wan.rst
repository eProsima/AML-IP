.. include:: ../exports/alias.include

.. _demo_inference:

########
WAN Demo
########

This demonstrator shows how to implement 2 kind of nodes:
:ref:`user_manual_nodes_inference` and :ref:`user_manual_nodes_edge`.
With these 2 nodes implemented, the user can deploy as many nodes of each kind as desired and check the
behavior of a simulated |amlip| network running.
They are implemented one in Python and one in C++ to demonstrate as well how to instantiate each kind of node
with different :term:`APIs <API>`, and to prove the communication between the 2 implementations.

Simulation
==========

TensorFlow
----------

TODO

Darknet
-------

TODO

Edge Node
---------

This node simulates a :ref:`user_manual_nodes_edge`.
It is implemented in |python| using :code:`amlip_py` API.
There are 2 different ways to run it, an automatic one and a manual one:

Inference Node
--------------

This node simulates a :ref:`user_manual_nodes_inference`.
It is implemented in |cpp| using :code:`amlip_cpp` API.

To run it, one string argument is required. This will be the model this node will use to make the inference.

Installation
============

First of all, check that :code:`amlip_demo_nodes` sub-package is correctly installed.
If it is not, please refer to :ref:`developer_manual_installation_sources_linux_colcon_demos`.

Requirements
============

The demo requires the following tools to be installed in the system:

.. code-block:: bash

    sudo apt install -y  swig espeak alsa-utils libopencv-dev ffmpeg
    pip3 install -U pyttsx3 opencv-python tensorflow tensorflow-hub tensorflow-object-detection-api pygame

Get the models
==============

Using TensorFlow
----------------

.. code-block:: bash
    cd ~/AML-IP-demos-ws/src/AML-IP-demos/amlip_demos/resource/tensorflow/models/
    wget -O centernet_hourglass_512x512_kpts_1.tar.gz https://tfhub.dev/tensorflow/centernet/hourglass_512x512_kpts/1?tf-hub-format=compressed
    mkdir centernet_hourglass_512x512_kpts_1
    tar -xvf centernet_hourglass_512x512_kpts_1.tar.gz -C ./centernet_hourglass_512x512_kpts_1

Using Darknet
-------------

.. code-block:: bash
    cd ~/AML-IP-demos-ws/src/AML-IP-demos/amlip_demos/resource/darknet/
    wget https://pjreddie.com/media/files/yolov3-tiny.weights

Run demo
========

The demo that is presented here follows the schema of the figure below:

.. figure:: /rst/figures/workload_distribution_basic_demo.png
    :width: 50%

Run Edge Node
-------------

Run the following command:

.. code-block:: bash

    # Source colcon installation
    source install/setup.bash

    # To execute Edge Node to send an image to inferred
    cd ~/AML-IP-demos-ws/src/AML-IP-demos/amlip_demos/amlip_demos
    python3 edge_node.py

Take into account that this node will wait until there is an *Inference Node* running and available
in the same :term:`LAN` in order to process the inference.
The expected output is the following:

.. code-block:: bash

    Edge Node AMLEdgeNode.e5.45.c7.f5 ready.
    Edge Node AMLEdgeNode.e5.45.c7.f5 sending data: /home/irenebm/annapurna/AML-IP/src/amlipdemos/amlip_demos/resource/tensorflow/models/research/object_detection/test_images/image2.jpg.
    Edge Node received inference from AMLInferenceNode.7c.df.34.e8
    Edge Node received inference kite: 81% person: 76% person: 74% kite: 72% kite: 71% person: 67% kite: 67% kite: 66% kite: 59% person: 54% person: 52% person: 52% person: 52% person: 51% kite: 50% person: 45% person: 45% surfboard: 40% surfboard: 39% surfboard: 31% kite: 30% person: 24% bird: 23% handbag: 23% person: 22% surfboard: 21% kite: 20% handbag: 20% kite: 20% surfboard: 19% person: 18% person: 18% kite: 16% person: 15% person: 15% person: 14% person: 14% surfboard: 13% kite: 13% backpack: 12% person: 12% kite: 11% person: 11% surfboard: 11% surfboard: 11% kite: 11%
    # ... Speak
    Edge Node AMLEdgeNode.e5.45.c7.f5 closing.


Run Inference Node with TensorFlow
----------------------------------

Run the following command to process the inference:

.. code-block:: bash

    # Source colcon installation
    source install/setup.bash

    # To execute Inference Node with pre-trained model from TensorFlow
    cd ~/AML-IP-demos-ws/src/AML-IP-demos/amlip_demos/amlip_demos
    python3 inference_node.py tensorflow

This execution expects an output similar to the one shown below:

.. code-block:: bash

    2023-02-14 14:50:42.711797: I tensorflow/core/platform/cpu_feature_guard.cc:193] This TensorFlow binary is optimized with oneAPI Deep Neural Network Library (oneDNN) to use the following CPU instructions in performance-critical operations:  AVX2 FMA
    To enable them in other operations, rebuild TensorFlow with the appropriate compiler flags.
    Inference Node AMLInferenceNode.7c.df.34.e8 ready.
    Model Handle at TensorFlow Hub: /home/irenebm/annapurna/AML-IP/src/amlipdemos/amlip_demos/resource/tensorflow/models/centernet_hourglass_512x512_kpts_1
    loading model...
    WARNING:absl:Importing a function (__inference_batchnorm_layer_call_and_return_conditional_losses_42408) with ops with unsaved custom gradients. Will likely fail if a gradient is requested.
    WARNING:absl:Importing a function (__inference_batchnorm_layer_call_and_return_conditional_losses_209416) with ops with unsaved custom gradients. Will likely fail if a gradient is requested.
    WARNING:absl:Importing a function (__inference_batchnorm_layer_call_and_return_conditional_losses_220336) with ops with unsaved custom gradients. Will likely fail if a gradient is requested.
    ...
    WARNING:absl:Importing a function (__inference_batchnorm_layer_call_and_return_conditional_losses_55827) with ops with unsaved custom gradients. Will likely fail if a gradient is requested.
    WARNING:absl:Importing a function (__inference_batchnorm_layer_call_and_return_conditional_losses_56488) with ops with unsaved custom gradients. Will likely fail if a gradient is requested.
    model loaded!
    Selected model:tensorflow
    2023-02-14 14:51:14.165305: W tensorflow/core/grappler/optimizers/loop_optimizer.cc:907] Skipping loop optimization for Merge node with control input: StatefulPartitionedCall/cond/then/_918/cond/Assert_2/AssertGuard/branch_executed/_1123
    inference ready!
    sending inference: kite: 81% person: 76% person: 74% kite: 72% kite: 71% person: 67% kite: 67% kite: 66% kite: 59% person: 54% person: 52% person: 52% person: 52% person: 51% kite: 50% person: 45% person: 45% surfboard: 40% surfboard: 39% surfboard: 31% kite: 30% person: 24% bird: 23% handbag: 23% person: 22% surfboard: 21% kite: 20% handbag: 20% kite: 20% surfboard: 19% person: 18% person: 18% kite: 16% person: 15% person: 15% person: 14% person: 14% surfboard: 13% kite: 13% backpack: 12% person: 12% kite: 11% person: 11% surfboard: 11% surfboard: 11% kite: 11%
    Inference sent to client AMLEdgeNode.e5.45.c7.f5.

Run Inference Node with Darknet
-------------------------------

Run the following command to answer 2 jobs before closing:

.. code-block:: bash

    # Source colcon installation
    source install/setup.bash

    # To execute Inference Node with pre-trained model from Darknet
    cd ~/AML-IP-demos-ws/src/AML-IP-demos/amlip_demos/amlip_demos
    python3 inference_node.py darknet

This execution expects an output similar to the one shown below:

.. code-block:: bash

    2023-02-14 14:39:23.253475: I tensorflow/core/platform/cpu_feature_guard.cc:193] This TensorFlow binary is optimized with oneAPI Deep Neural Network Library (oneDNN) to use the following CPU instructions in performance-critical operations:  AVX2 FMA
    To enable them in other operations, rebuild TensorFlow with the appropriate compiler flags.
    Inference Node AMLInferenceNode.2a.73.87.3d ready.
    layer     filters    size              input                output
        0 conv     32  3 x 3 / 1   608 x 608 x   3   ->   608 x 608 x  32  0.639 BFLOPs
        1 conv     64  3 x 3 / 2   608 x 608 x  32   ->   304 x 304 x  64  3.407 BFLOPs
        2 conv     32  1 x 1 / 1   304 x 304 x  64   ->   304 x 304 x  32  0.379 BFLOPs
        3 conv     64  3 x 3 / 1   304 x 304 x  32   ->   304 x 304 x  64  3.407 BFLOPs
        4 res    1                 304 x 304 x  64   ->   304 x 304 x  64
        5 conv    128  3 x 3 / 2   304 x 304 x  64   ->   152 x 152 x 128  3.407 BFLOPs
        ...
        102 conv    256  3 x 3 / 1    76 x  76 x 128   ->    76 x  76 x 256  3.407 BFLOPs
        103 conv    128  1 x 1 / 1    76 x  76 x 256   ->    76 x  76 x 128  0.379 BFLOPs
        104 conv    256  3 x 3 / 1    76 x  76 x 128   ->    76 x  76 x 256  3.407 BFLOPs
        105 conv    255  1 x 1 / 1    76 x  76 x 256   ->    76 x  76 x 255  0.754 BFLOPs
        106 yolo
    Loading weights from /home/irenebm/annapurna/AML-IP/src/amlipdemos/amlip_demos/resource/darknet/yolov3.weights...Done!
    Selected model:darknet
    inference ready!
    sending inference: person: 99.9% person: 99.64% kite: 98.59% person: 97.49% person: 95.67% person: 94.98% person: 91.02% person: 88.12% person: 84.85% kite: 84.32% kite: 79.83% kite: 73.29% person: 52.15%
    Inference sent to client AMLEdgeNode.61.33.82.fe.
