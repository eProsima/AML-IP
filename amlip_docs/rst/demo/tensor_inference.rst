.. include:: ../exports/alias.include

.. _demo_inference:

#########################
TensorFlow Inference Demo
#########################

.. contents::
    :local:
    :backlinks: none
    :depth: 2

Background
==========

This demo shows how to implement 2 types of nodes, :ref:`user_manual_nodes_inference` and :ref:`user_manual_nodes_edge`, to perform TensorFlow inference on a given image.

With these 2 nodes implemented, the user can deploy as many nodes of each kind as desired and check the behaviour of a simulated |amlip| network running.


Inference refers to the process of using a trained model to make predictions or draw conclusions based on input data.
It involves applying the learned knowledge and statistical relationships encoded in the model to new, unseen data.

The inference of an image involves passing the image through a trained AI model to obtain a classification based on the learned knowledge and patterns within the model.

TensorFlow
----------

`TensorFlow <https://www.tensorflow.org/>`_ is an end-to-end machine learning platform with pre-trained models.

Edge Node
---------

This node simulates an :ref:`user_manual_nodes_edge`.
It is implemented in |python| using :code:`amlip_py` API.

Inference Node
--------------

This node simulates an :ref:`user_manual_nodes_inference`.
It is implemented in |python| using :code:`amlip_py` API.
To run it, one string argument is required. This will be the model this node will use to make the inference.

Prerequisites
=============

First of all, check that :code:`amlip_demo_nodes` sub-package is correctly installed.
If it is not, please refer to :ref:`developer_manual_installation_sources_linux`.


The demo requires the following tools to be installed in the system:

.. code-block:: bash

    sudo apt install -y  swig espeak alsa-utils libopencv-dev ffmpeg
    pip3 install -U pyttsx3 opencv-python pygame

Ensure that you have TensorFlow and TensorFlow Hub installed in your Python environment before proceeding.
You can install them using pip by executing the following commands:

.. code-block:: bash

    pip3 install tensorflow tensorflow-hub tensorflow-object-detection-api

Additionally, it is required to obtain the TensorFlow model from `TensorFlow Hub <https://tfhub.dev/>`_, follow the steps below:

.. code-block:: bash

    cd ~/AML-IP-demos-ws/src/AML-IP-demos/amlip_demos/resource/tensorflow/models/
    wget -O centernet_hourglass_512x512_kpts_1.tar.gz https://tfhub.dev/tensorflow/centernet/hourglass_512x512_kpts/1?tf-hub-format=compressed
    mkdir centernet_hourglass_512x512_kpts_1
    tar -xvf centernet_hourglass_512x512_kpts_1.tar.gz -C ./centernet_hourglass_512x512_kpts_1

Run demo
========

The demo that is presented here follows the schema of the figure below:

.. figure:: /rst/figures/tensor_inference_demo.png

Run Edge Node
-------------

In the first terminal, run the following command:

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


Run Inference Node
------------------

In the second terminal, run the following command to process the inference:

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
    Model Handle at TensorFlow Hub: /home/user/AML-IP/src/amlipdemos/amlip_demos/resource/tensorflow/models/centernet_hourglass_512x512_kpts_1
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
