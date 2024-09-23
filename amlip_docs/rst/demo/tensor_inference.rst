.. include:: ../exports/alias.include

.. _demo_inference:

####################
TensorFlow Inference
####################

.. contents::
    :local:
    :backlinks: none
    :depth: 2

Background
==========

Inference is the process of using a trained model to make predictions or draw conclusions from new, unseen data.
It involves applying the learned knowledge and statistical relationships encoded in the model to the input data.
When inferring an image, the image is passed through a trained AI model to classify it based on the patterns and knowledge the model has learned.

This demo shows how to implement 2 types of nodes, :ref:`user_manual_nodes_inference` and :ref:`user_manual_nodes_edge`, to perform TensorFlow inference on a given image.
With these 2 nodes implemented, the user can deploy as many nodes of each kind as desired and check the behavior of a simulated |amlip| network running.

The demo that is presented here follows the schema of the figure below:

.. figure:: /rst/figures/demos/tensor_inference_demo.png

* `TensorFlow <https://www.tensorflow.org/>`_ is an end-to-end machine learning platform with pre-trained models.


* Edge Node simulates an :ref:`user_manual_nodes_edge`. It is implemented in |python| using :code:`amlip_py` API.

* Inference Node simulates an :ref:`user_manual_nodes_inference`. It is implemented in |python| using :code:`amlip_py` API.

Prerequisites
=============

First of all, check that :code:`amlip_tensorflow_inference_demo` sub-package is correctly installed.
If it is not, please refer to :ref:`developer_manual_installation_sources_linux_colcon_demos`.

The demo requires the following tools to be installed in the system:

.. code-block:: bash

    sudo apt install -y  swig alsa-utils libopencv-dev
    pip3 install -U pyttsx3 opencv-python
    python -m venv aml-ip-venv
    source aml-ip-venv/bin/activate

Ensure that you have TensorFlow and TensorFlow Hub installed in your Python environment before proceeding.
You can install them by executing the following commands:

.. code-block:: bash

    python3 -m pip install tensorflow[and-cuda]
    pip3 install tensorflow-hub tensorflow-object-detection-api protobuf==3.20

Additionally, it is required to obtain the TensorFlow model from `TensorFlow Hub <https://tfhub.dev/>`_, follow the steps below:

.. code-block:: bash

    cd ~/AML-IP-ws/src/AML-IP/amlip_demo_nodes/amlip_tensorflow_inference_demo/resource/tensorflow/models/
    wget -O centernet_hourglass_512x512_kpts_1.tar.gz https://tfhub.dev/tensorflow/centernet/hourglass_512x512_kpts/1?tf-hub-format=compressed
    mkdir centernet_hourglass_512x512_kpts_1
    tar -xvf centernet_hourglass_512x512_kpts_1.tar.gz -C ./centernet_hourglass_512x512_kpts_1

Building the demo
=================

To build the demo, build the packages with Colcon:

.. code-block:: bash

    colcon build --packages-up-to amlip_tensorflow_inference_demo

Once AML-IP packages are installed and built, import the libraries using the following command.

.. code-block:: bash

    source install/setup.bash

Explaining the demo
===================

In this section, we will explore and explain the demo in detail.

Edge Node
---------

Edge Node serves as the entity responsible for sending the data to be inferred to the Inference Node.
The Edge Node is typically located at the edge of a network or closer to the data source, such as a sensor or a device generating the data.

This is the Python code for the Edge Node application.
This code can be found `here <https://github.com/eProsima/AML-IP/blob/main/amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo/edge_node_async.py>`__.

The next block includes the Python header files that allow the use of the AML-IP Python API.

.. literalinclude:: /../amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo/edge_node_async.py
  :language: python
  :lines: 18-19

Let's continue explaining the global variables.
The ``waiter`` object is used to pause the node's execution until the inference result is received.
``DOMAIN_ID`` allows the execution to be isolated because only DomainParticipants with the same Domain Id would be able to communicate to each other.

.. literalinclude:: /../amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo/edge_node_async.py
  :language: python
  :lines: 25-29

The definition of the ``inference_received`` function prints the details of the received inference.

.. literalinclude:: /../amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo/edge_node_async.py
  :language: python
  :lines: 32-38

We define the ``main`` function.

.. literalinclude:: /../amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo/edge_node_async.py
  :language: python
  :lines: 41

First, we create an instance of ``AsyncEdgeNode``.
The first thing the constructor gets is the given name.
Then a `listener <https://fast-dds.docs.eprosima.com/en/stable/fastdds/dds_layer/core/entity/entity.html#listener>`__,
which is an ``InferenceListenerLambda`` object, is created for the function ``inference_received`` declared above.
The listener acts as an asynchronous notification system that allows the entity to notify the application about the Status changes in the entity.
This function is called each time an inference is received.
Lastly, a ``DOMAIN_ID`` is specified, which allows the execution to be isolated.

.. literalinclude:: /../amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo/edge_node_async.py
  :language: python
  :lines: 43-46

The next code block loads the image using ``cv2.imread`` based on the specified `image_path <https://github.com/eProsima/AML-IP/blob/main/amlip_demo_nodes/amlip_tensorflow_inference_demo/resource/tensorflow/models/research/object_detection/test_images/dog.jpg>`__.
It converts the size information and the image into bytes and combines the two to send them to the Inference node.

.. literalinclude:: /../amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo/edge_node_async.py
  :language: python
  :lines: 51-65

Next, the ``request_inference`` method is invoked to send the image for inference.

.. literalinclude:: /../amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo/edge_node_async.py
  :language: python
  :lines: 69

Finally, the program waits for the inference solution using ``waiter.wait``.

.. literalinclude:: /../amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo/edge_node_async.py
  :language: python
  :lines: 74

Once the solution is received, the execution finishes.

Inference Node
--------------

The Inference Node is responsible for making the inferences or predictions on the data it receives using a TensorFlow model.
The Inference Node is typically a server or a computing resource equipped with high-performance hardware optimized for executing machine learning models efficiently.

This is the Python code for the Inference Node application.
This code can be found `here <https://github.com/eProsima/AML-IP/blob/main/amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo/inference_node_async.py>`__.

The next block includes the Python header files that allow the use of the AML-IP Python API.

.. literalinclude:: /../amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo/inference_node_async.py
  :language: python
  :lines: 19-20

Let's continue explaining the global variables.
The ``DOMAIN_ID`` variable allows the execution to be isolated because only DomainParticipants with the same Domain Id would be able to communicate to each other.
The ``tolerance`` variable sets a threshold to filter out detections with a probability lower than the specified tolerance value.

.. literalinclude:: /../amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo/inference_node_async.py
  :language: python
  :lines: 28-32

The model is loaded from TensorFlow using the specified path.

.. literalinclude:: /../amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo/inference_node_async.py
  :language: python
  :lines: 34-45

The ``process_inference`` function is responsible for computing the inference when data is received.
Inference is performed using the input data and the loaded model.
Note that detected objects are filtered based on the specified tolerance.

.. literalinclude:: /../amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo/inference_node_async.py
  :language: python
  :lines: 49-79

We define the ``main`` function.

.. literalinclude:: /../amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo/inference_node_async.py
  :language: python
  :lines: 82

We create an instance of ``AsyncInferenceNode``.
The first thing the constructor gets is the name ``AMLInferenceNode``.
Then a `listener <https://fast-dds.docs.eprosima.com/en/stable/fastdds/dds_layer/core/entity/entity.html#listener>`__,
which is an ``InferenceReplierLambda`` object, is created for the function ``process_inference`` declared above.
This means that the ``process_inference`` function will be called to handle the inference requests.
Additionally, the domain is specified using the DOMAIN_ID variable.

.. literalinclude:: /../amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo/inference_node_async.py
  :language: python
  :lines: 84-87

This initiates the Inference Node, which will listen for incoming inference requests and invoke the ``process_inference`` function to handle them.

.. literalinclude:: /../amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo/inference_node_async.py
  :language: python
  :lines: 91

Finally, the node waits for a SIGINT signal (``Ctrl+C``) to stop and close gracefully.

.. literalinclude:: /../amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo/inference_node_async.py
  :language: python
  :lines: 93-98

Run demo
========

This demo explains the implemented nodes in `amlip_demo_nodes/amlip_tensorflow_inference_demo`.

Run Edge Node
-------------

In one terminal, run the Edge Node with the following command:

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

In a second terminal, run the following command to process the inference:

.. code-block:: bash

    # Source colcon installation
    source install/setup.bash

    # To execute Inference Node with pre-trained model from TensorFlow
    cd ~/AML-IP-ws/src/AML-IP/amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo
    python3 inference_node_async.py

The execution expects an output similar to the one shown below:

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

.. warning:: If you encounter an output similar to the next one, follow the set of instructions outlined :ref:`below <demo_inference_troubleshooting>`:

    .. code-block:: bash

        terminate called after throwing an instance of 'Swig::DirectorMethodException'
            what():  SWIG director method error. In method 'process_inference': AttributeError: module 'tensorflow' has no attribute 'gfile'
        Aborted (core dumped)

Results
-------

Based on the information acquired, we have successfully generated the next image:

.. figure:: /rst/figures/demos/inferred_image.png


Run multiple nodes of each kind
===============================

One of the advantages inherent to this architecture lies in its ability to support multiple models operating concurrently across multiple :ref:`user_manual_nodes_inference`, while simultaneously requesting inferences from :ref:`user_manual_nodes_edge` in parallel.
This architectural design fosters a highly efficient and scalable system, enabling the execution of diverse inference tasks in a distributed manner.

How to use your own model
=========================

To use your own model, simply download it and load it by passing the path to the function:

.. code-block:: python

    hub_model = hub.load(your_model_path)

.. _demo_inference_troubleshooting:

Troubleshooting
===============

TensorFlow using old API
------------------------

Please be aware that Simple TensorFlow Serving is currently not compatible with TensorFlow 2.0 due to its reliance on the older API.
It is important to note that in TensorFlow 2.0, the `gfile` package has been relocated under the `tf.io` module.
Therefore, if you intend to utilize TensorFlow 2.0, please take into consideration this change in the package structure and update your code accordingly.
Check following `issue <https://github.com/tensorflow/tensorflow/issues/31315>`_ for further information.

To update the code, please follow these `steps <https://stackoverflow.com/questions/55591437/attributeerror-module-tensorflow-has-no-attribute-gfile>`_:

1. Locate the file `label_map_util.py`. (default path: ``.local/lib/python3.x/site-packages/object_detection/utils/label_map_util.py``)
2. Navigate to line 132 within the file.
3. Replace `tf.gfile.GFile` with `tf.io.gfile.GFile`.

Next Steps
==========

Now you can develop more functionalities in your application.
See also :ref:`this tutorial <demo_rosbot2r_inference>` which explains how to take the image from a ROSbot 2R Camera.
