.. include:: ../exports/alias.include

.. _demo_collaborative_learning:

######################
Collaborative Learning
######################

.. contents::
    :local:
    :backlinks: none
    :depth: 2

Background
==========

This demo shows a :ref:`user_manual_scenarios_collaborative_learning` and the |amlip| nodes involved:
:ref:`user_manual_nodes_model_receiver` and :ref:`user_manual_nodes_model_sender`.
With these 2 nodes implemented, the user can deploy as many nodes of each kind as desired and check the
behavior of a simulated |amlip| network running.
They are implemented in Python to prove the communication between the 2 implementations.

The purpose of the demo is to show how a *Sender* and a *Receiver* node can communicate.
The *Receiver* node awaits model statistics from the *Sender*.
Since the *Sender* doesn't have a real *AML Engine*, it sends the model statistics as a string.
Upon receiving the statistics, the *Receiver* sends a model request, also as a string since it doesn't have an *AML Engine*.
Then, the *Sender* converts the received model request to uppercase and sends it back as a model reply.

.. figure:: /rst/figures/demos/collaborative_demo.png
    :align: center
    :width: 80%

Prerequisites
=============

Before running this demo, ensure that :code:`AML-IP` is correctly installed using one of the following installation methods:

* :ref:`installation_manual_linux`
* :ref:`installation_manual_windows`
* :ref:`docker`

Building the demo
=================

If the demo package is not compiled, please refer to :ref:`developer_manual_installation_sources_linux_colcon_demos` or run the command below.

.. code-block:: bash

    colcon build --packages-up-to amlip_collaborative_learning_demo

Once AML-IP packages are installed and built, import the libraries using the following command.

.. code-block:: bash

    source install/setup.bash

Explaining the demo
===================

In this section, we will delve into the details of the demo and how it works.

Model Manager Receiver Node
---------------------------

This is the Python code for the :ref:`user_manual_nodes_model_receiver` application.
It does not use real *AML Models*, but strings.
It is implemented in |python| using :code:`amlip_py` API.

This code can be found `here <https://github.com/eProsima/AML-IP/blob/main/amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo/model_receiver_custom.py>`__.

The next block includes the Python header files that allow the use of the AML-IP Python API.

.. literalinclude:: /../amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo/model_receiver_custom.py
  :language: python
  :lines: 17-21

Let's continue explaining the global variables.

``DOMAIN_ID`` variable isolates the execution within a specific domain. Nodes with the same domain ID can communicate with each other.

.. literalinclude:: /../amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo/model_receiver_custom.py
  :language: python
  :lines: 24

``waiter`` is a ``WaitHandler`` , which is an object that allows multiple threads wait, until another thread awakes them. In this case, due to being a ``BoolWaitHandler``, it waits on a boolean value.
Whenever this value is ``True``, threads awake.
Whenever it is ``False``, threads wait.

.. literalinclude:: /../amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo/model_receiver_custom.py
  :language: python
  :lines: 27

The ``CustomModelListener`` class listens to :ref:`user_manual_scenarios_collaborative_learning_statistics` and :ref:`user_manual_scenarios_collaborative_learning_model_reply` messages received from a :ref:`user_manual_nodes_model_sender`.
This class is supposed to be implemented by the user in order to process the messages received from other nodes in the network.

.. literalinclude:: /../amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo/model_receiver_custom.py
  :language: python
  :lines: 30-50

The `main` function orchestrates the execution of the Model Manager Receiver node. It creates an instance of the `ModelManagerReceiverNode` and starts its execution with the specified listener.

.. literalinclude:: /../amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo/model_receiver_custom.py
   :language: python
   :lines: 53-73

After starting the node, it waits for statistics to arrive from the :ref:`user_manual_nodes_model_sender`.

.. literalinclude:: /../amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo/model_receiver_custom.py
  :language: python
  :lines: 75-76

Then, it requests a model from the :ref:`user_manual_nodes_model_sender` using the received server ID.

.. literalinclude:: /../amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo/model_receiver_custom.py
  :language: python
  :lines: 78-79

Finally, the node stops.

.. literalinclude:: /../amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo/model_receiver_custom.py
  :language: python
  :lines: 81

Model Manager Sender Node
-------------------------

This is the Python code for the :ref:`user_manual_nodes_model_sender` application.
It does not use real *AML Models* nor does it have a real *AML Engine* Instead, strings are sent and the calculation is an *upper-case* conversion of the string received.
It is implemented in |python| using :code:`amlip_py` API.

This code can be found `here <https://github.com/eProsima/AML-IP/blob/main/amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo/model_sender_custom.py>`__.

The following block includes the Python header files necessary for using the AML-IP Python API.

.. literalinclude:: /../amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo/model_sender_custom.py
  :language: python
  :lines: 17-20

Let's continue explaining the global variables.

``DOMAIN_ID`` isolates the execution within a specific domain. Nodes with the same domain ID can communicate with each other.

.. literalinclude:: /../amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo/model_sender_custom.py
  :language: python
  :lines: 23

``waiter`` is a ``WaitHandler`` , which is an object that allows multiple threads wait, until another thread awakes them. In this case, due to being a ``BoolWaitHandler``, it waits on a boolean value.
Whenever this value is ``True``, threads awake.
Whenever it is ``False``, threads wait.

.. literalinclude:: /../amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo/model_sender_custom.py
  :language: python
  :lines: 26

The ``CustomModelReplier`` class listens to :ref:`user_manual_scenarios_collaborative_learning_model_request` request messages received from a :ref:`user_manual_nodes_model_receiver`.
This class is supposed to be implemented by the user in order to process the messages.

.. literalinclude:: /../amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo/model_sender_custom.py
  :language: python
  :lines: 29-43

The `main` function orchestrates the execution of the Model Manager Sender node. It creates an instance of `ModelManagerSenderNode`.

.. literalinclude:: /../amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo/model_sender_custom.py
  :language: python
  :lines: 46-56

After starting the node, it publishes statistics using the ``publish_statistics()`` function, which fills a :ref:`user_manual_scenarios_collaborative_learning_statistics`  and publishes it.

.. literalinclude:: /../amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo/model_sender_custom.py
  :language: python
  :lines: 61-63

Then we start the node execution, passing the previously defined ``CustomModelReplier()`` class, which is responsible for managing the request received.

.. literalinclude:: /../amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo/model_sender_custom.py
  :language: python
  :lines: 65-66

Waits for the response model to be sent to the :ref:`user_manual_nodes_model_receiver`.

.. literalinclude:: /../amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo/model_sender_custom.py
  :language: python
  :lines: 68-69

Finally, it stops and closes the node.

.. literalinclude:: /../amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo/model_sender_custom.py
  :language: python
  :lines: 71

Running the demo
================

This demo runs the implemented nodes in ``amlip_demo_nodes/amlip_collaborative_learning_demo``.

Run Model Manager Receiver Node
-------------------------------

Run the following command:

.. code-block:: bash

    # Source colcon installation
    source install/setup.bash

    # To execute Model Manager Receiver Node
    cd ~/AML-IP-ws/src/AML-IP/amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo
    python3 model_receiver_custom.py

The expected output is the following:

.. code-block:: bash

    Starting Manual Test Model Manager Receiver Node Py execution. Creating Node...
    Node created: ModelManagerReceiver.0f.19.23.2d. Already processing models.
    Model reply received from server
    solution: MOBILENET V1
    Finishing Manual Test Model Manager Receiver Node Py execution.


Run Model Manager Sender Node
-----------------------------

Run the following command to answer before closing:

.. code-block:: bash

    # Source colcon installation
    source install/setup.bash

    # To execute Model Manager Sender Node
    cd ~/AML-IP-ws/src/AML-IP/amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo
    python3 model_sender_custom.py

This execution expects an output similar to the one shown below:

.. code-block:: bash

    Starting Manual Test Model Manager Sender Node Py execution. Creating Node...
    Node created: ModelManagerSender.0a.14.1e.28. Already processing models.
    Model request received from client
    model: MobileNet V1
    solution: MOBILENET V1
    Finishing Manual Test Model Manager Sender Node Py execution.
