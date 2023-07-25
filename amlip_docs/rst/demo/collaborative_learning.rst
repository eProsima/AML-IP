.. include:: ../exports/alias.include

.. _demo_collaborative_learning:

######################
Collaborative Learning
######################

This demonstrator shows how to implement 2 kind of nodes:
:ref:`user_manual_nodes_model_receiver` and :ref:`user_manual_nodes_model_sender`.
With these 2 nodes implemented, the user can deploy as many nodes of each kind as desired and check the
behavior of a simulated |amlip| network running.
They are implemented one in Python and one in C++ to demonstrate as well how to instantiate each kind of node
with different :term:`APIs <API>`, and to prove the communication between the 2 implementations.

Simulation
==========

Model Manager Receiver Node
---------------------------

This node simulates a :ref:`user_manual_nodes_model_receiver`.
It does not use real *AML Models*, but strings.
It is implemented in |python| using :code:`amlip_py` API.

Model Manager Sender Node
-------------------------

This node simulates a :ref:`user_manual_nodes_model_sender`.
It does not use real *AML Models*, but strings.
It does not have a real *AML Engine* but instead the calculation is an *upper-case* conversion of the string received.
It is implemented in |python| using :code:`amlip_py` API.

Installation
============

First of all, check that :code:`amlip_demo_nodes` sub-package is correctly installed.
If it is not, please refer to :ref:`developer_manual_installation_sources_linux_colcon_demos`.

Run demo
========

Run Model Manager Receiver Node
-------------------------------

Run the following command:

.. code-block:: bash

    # Source colcon installation
    source install/setup.bash

    # To execute Main Node to send 2 jobs
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

    # To execute Main Node to send 2 jobs
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
