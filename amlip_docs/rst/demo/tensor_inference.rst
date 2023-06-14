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

Inference refers to the process of using a trained model to make predictions or draw conclusions based on input data.
It involves applying the learned knowledge and statistical relationships encoded in the model to new, unseen data.
The inference of an image involves passing the image through a trained AI model to obtain a classification based on the learned knowledge and patterns within the model.

This demo shows how to implement 2 types of nodes, :ref:`user_manual_nodes_inference` and :ref:`user_manual_nodes_edge`, to perform TensorFlow inference on a given image.
With these 2 nodes implemented, the user can deploy as many nodes of each kind as desired and check the behavior of a simulated |amlip| network running.

The demo that is presented here follows the schema of the figure below:

.. figure:: /rst/figures/tensor_inference_demo.png

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
If it is not, please refer to :ref:`developer_manual_installation_sources_linux_colcon_demos`.


The demo requires the following tools to be installed in the system:

.. code-block:: bash

    sudo apt install -y  swig espeak alsa-utils libopencv-dev ffmpeg
    pip3 install -U pyttsx3 opencv-python pygame
    curl https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -o Miniconda3-latest-Linux-x86_64.sh
    bash Miniconda3-latest-Linux-x86_64.sh
    # For changes to take effect, close and re-open your current shell.
    conda create --name tf python=3.9
    conda install -c conda-forge cudatoolkit=11.8.0
    mkdir -p $CONDA_PREFIX/etc/conda/activate.d
    echo 'CUDNN_PATH=$(dirname $(python3 -c "import nvidia.cudnn;print(nvidia.cudnn.__file__)"))' >> $CONDA_PREFIX/etc/conda/activate.d/env_vars.sh
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CONDA_PREFIX/lib/:$CUDNN_PATH/lib' >> $CONDA_PREFIX/etc/conda/activate.d/env_vars.sh
    source $CONDA_PREFIX/etc/conda/activate.d/env_vars.sh

Ensure that you have TensorFlow and TensorFlow Hub installed in your Python environment before proceeding.
You can install them using pip by executing the following commands:

.. code-block:: bash

    pip3 install tensorflow tensorflow-hub tensorflow-object-detection-api nvidia-cudnn-cu11==8.6.0.163

Additionally, it is required to obtain the TensorFlow model from `TensorFlow Hub <https://tfhub.dev/>`_, follow the steps below:

.. code-block:: bash

    cd ~/AML-IP-ws/src/AML-IP/amlip_demo_nodes/amlip_tensorflow_inference_demo/resource/tensorflow/models/
    wget -O centernet_hourglass_512x512_kpts_1.tar.gz https://tfhub.dev/tensorflow/centernet/hourglass_512x512_kpts/1?tf-hub-format=compressed
    mkdir centernet_hourglass_512x512_kpts_1
    tar -xvf centernet_hourglass_512x512_kpts_1.tar.gz -C ./centernet_hourglass_512x512_kpts_1

Run demo
========

Run Edge Node
-------------

In the first terminal, run the following command:

.. code-block:: bash

    # Source colcon installation
    source install/setup.bash

    # To execute Edge Node to send an image to inferred
    cd ~/AML-IP-ws/src/amlip/amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo
    python3 edge_node_async.py

Take into account that this node will wait until there is an *Inference Node* running and available
in the same :term:`LAN` in order to process the inference.
The expected output is the following:

.. code-block:: bash

    Edge Node AMLEdgeNode.fb.d4.38.13 ready.
    Edge Node AMLEdgeNode.fb.d4.38.13 sending data.

    Edge Node received inference from AMLInferenceNode.b8.34.4d.a3
    Edge Node received inference:
    Box [(0.15590962767601013, 0.21641747653484344), (0.7388607263565063, 0.7326743006706238)] bicycle: 97%
    Box [(0.16968876123428345, 0.38129815459251404), (0.403958797454834, 0.9422630071640015)] dog: 92%
    Box [(0.6158109307289124, 0.13117200136184692), (0.9053990244865417, 0.2978983521461487)] truck: 53%
    Box [(0.6158109307289124, 0.13117200136184692), (0.9053990244865417, 0.2978983521461487)] car: 48%
    Box [(0.8892407417297363, 0.19558095932006836), (0.933372974395752, 0.2684069573879242)] potted plant: 34%
    Box [(0.0753115713596344, 0.15651819109916687), (0.13415342569351196, 0.22736744582653046)] motorcycle: 32%

    Edge Node AMLEdgeNode.fb.d4.38.13 closing.


Run Inference Node
------------------

In the second terminal, run the following command to process the inference:

.. code-block:: bash

    # Source colcon installation
    source install/setup.bash

    # To execute Inference Node with pre-trained model from TensorFlow
    cd ~/AML-IP-ws/src/AML-IP/amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo
    python3 inference_node_async.py

This execution expects an output similar to the one shown below:

.. code-block:: bash

    2023-02-14 14:50:42.711797: I tensorflow/core/platform/cpu_feature_guard.cc:193] This TensorFlow binary is optimized with oneAPI Deep Neural Network Library (oneDNN) to use the following CPU instructions in performance-critical operations:  AVX2 FMA
    To enable them in other operations, rebuild TensorFlow with the appropriate compiler flags.
    Inference Node AMLInferenceNode.b8.34.4d.a3 ready.
    Model Handle at TensorFlow Hub: /home/user/AML-IP-ws/src/AML-IP/amlip_demo_nodes/amlip_tensorflow_inference_demo/resource/tensorflow/models/centernet_hourglass_512x512_kpts_1
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
    sending inference:
    Box [(0.15590962767601013, 0.21641747653484344), (0.7388607263565063, 0.7326743006706238)] bicycle: 97%
    Box [(0.16968876123428345, 0.38129815459251404), (0.403958797454834, 0.9422630071640015)] dog: 92%
    Box [(0.6158109307289124, 0.13117200136184692), (0.9053990244865417, 0.2978983521461487)] truck: 53%
    Box [(0.6158109307289124, 0.13117200136184692), (0.9053990244865417, 0.2978983521461487)] car: 48%
    Box [(0.8892407417297363, 0.19558095932006836), (0.933372974395752, 0.2684069573879242)] potted plant: 34%
    Box [(0.0753115713596344, 0.15651819109916687), (0.13415342569351196, 0.22736744582653046)] motorcycle: 32%

    Inference sent to client AMLEdgeNode.fb.d4.38.13.


Next steps
----------

Based on the information acquired, we have successfully generated the next image:

.. figure:: /rst/figures/inferred_image.png


Run multiple nodes of each kind
===============================

One of the advantages inherent to this architecture lies in its ability to support multiple models operating concurrently across multiple :ref:`user_manual_nodes_inference`, while simultaneously requesting inferences from :ref:`user_manual_nodes_edge` in parallel.
This architectural design fosters a highly efficient and scalable system, enabling the execution of diverse inference tasks in a distributed manner.

How to use your own model
=========================

To use your own model, simply download it and load it by passing the path to the function:

.. code-block:: python

    hub_model = hub.load(your_model_path)
