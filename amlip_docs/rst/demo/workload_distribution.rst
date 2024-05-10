.. include:: ../exports/alias.include

.. _demo_workload_distribution:

#####################
Workload Distribution
#####################

This demonstrator shows how to implement 2 kind of nodes:
:ref:`user_manual_nodes_computing` and :ref:`user_manual_nodes_main`.
With these 2 nodes implemented, the user can deploy as many nodes of each kind as desired and check the
behavior of a simulated |amlip| network running.
They are implemented one in Python and one in C++ to demonstrate as well how to instantiate each kind of node
with different :term:`APIs <API>`, and to prove the communication between the 2 implementations.

Simulation
==========

AML Mock
--------

For this demo the actual :term:`AML` Engine is not provided, and it is mocked.
This *Mock* simulates a difficult calculation by converting a string to uppercase and randomly waiting between
1 and 5 seconds in doing so.

Main Node
---------

This node simulates a :ref:`user_manual_nodes_main`.
It does not use real *AML Jobs*, but strings.
It is implemented in |python| using :code:`amlip_py` API.
There are 2 different ways to run it, an automatic one and a manual one:

Automatic version
^^^^^^^^^^^^^^^^^

In this version, the python executable expects input arguments.
For each argument, it will convert it to a string (:code:`str`) and send it as a *Job*.
Once the arguments run out, it will finish execution and destroy the Node.

Manual version
^^^^^^^^^^^^^^

In this version the python program expects to receive keyboard input.
For each keyboard input received, it will convert it to a string (:code:`str`) and send it as a *Job*.
When empty string given, it will finish execution and destroy the Node.

Computing Node
--------------

This node simulates a :ref:`user_manual_nodes_computing`.
It does not use real *AML Jobs*, but strings.
It does not have a real *AML Engine* but instead the calculation is an *upper-case* conversion of the string received.
It is implemented in |cpp| using :code:`amlip_cpp` API.

To run it, one integer argument is required. This will be the number of jobs this Node will answer to before finishing its execution and being destroyed.


Installation
============

First of all, check that :code:`amlip_demo_nodes` sub-package is correctly installed.
If it is not, please refer to :ref:`developer_manual_installation_sources_linux_colcon_demos`.


Run demo
========

The demo that is presented here follows the schema of the figure below:

.. figure:: /rst/figures/demos/workload_distribution_basic_demo.png
    :width: 50%

Run Main Node
-------------

Run the following command:

.. code-block:: bash

    # Source colcon installation
    source install/setup.bash

    # To execute Main Node to send 2 jobs
    python3 ./install/amlip_demo_nodes/bin/main_node.py first_job "second job"

Take into account that this node will wait until there are *Computing Nodes* running and available
in the same :term:`LAN` in order to solve the jobs.
The expected output is the following:

.. code-block:: bash

    Main Node AMLMainNode.aa.a5.47.fe ready.
    Main Node AMLMainNode.aa.a5.47.fe sending task <first_job>.
    # ... Waits for Computing Node
    Main Node received solution from AMLComputingNode.d1.c3.86.0a for job <first_job> => <FIRST_JOB>.
    Main Node AMLMainNode.aa.a5.47.fe sending task <second job>.
    Main Node received solution from AMLComputingNode.d1.c3.86.0a for job <second job> => <SECOND JOB>.
    Main Node AMLMainNode.aa.a5.47.fe closing.


Run Computing Node
------------------

Run the following command to answer 2 jobs before closing:

.. code-block:: bash

    # Source colcon installation
    source install/setup.bash

    # To execute Computing Node to answer 2 jobs
    ./install/amlip_demo_nodes/bin/computing_node 2

Take into account that this node will wait until it has solved 2 different jobs.
If there are more than 1 *Computing Node* running, one job is only solved by one of them.
This execution expects an output similar to the one shown below:

.. code-block:: bash

    Computing Node ID{AMLComputingNode.d1.c3.86.0a} computing 2 tasks.
    # ... Waits for Main Node
     Received Job: <first_job>. Processing...
     Answering Solution: <FIRST_JOB>.
    Computing Node ID{AMLComputingNode.d1.c3.86.0a} answered task. 1 remaining.
     Received Job: <second job>. Processing...
     Answering Solution: <SECOND JOB>.
    Computing Node ID{AMLComputingNode.d1.c3.86.0a} answered task. 0 remaining.
    Computing Node ID{AMLComputingNode.d1.c3.86.0a} closing.

Bigger scenarios
================

There is no limit in the number of nodes of each kind that could run in the same network.
However, take into account that these nodes are not meant to close nicely if they do not finish their tasks correctly,
thus calculate the number of jobs sent in order for all nodes to close gently.
