.. include:: ../exports/alias.include

.. _demo_workload_distribution:

#####################
Workload Distribution
#####################

.. contents::
    :local:
    :backlinks: none
    :depth: 2

Background
==========

This demonstrator showcases a :ref:`user_manual_scenarios_workload_distribution` and the |amlip| nodes involved: :ref:`user_manual_nodes_computing` and :ref:`user_manual_nodes_main`.
By implementing these two nodes, users can deploy as many nodes of each kind as desired and check the
behavior of a simulated |amlip| network running.
The nodes are implemented in both Python and C++, illustrating how to instantiate each type of node using different
:term:`APIs <API>` and demonstrating communication between the two implementations.

The purpose of this demo is to illustrate how a *Main Node* dispatches jobs to a *Computing Node* and how the *Computing Node* processes them.
The *Main Node* waits until a *Computing Node* is available to handle the job, while the *Computing Node* awaits a job to solve.

In this demo, the actual :term:`AML` Engine is not provided, and it is mocked.
This *Mock* simulates a difficult calculation by converting a string to uppercase and randomly waiting between
1 and 5 seconds in doing so.

The demo follows the schema depicted in the figure below:

.. figure:: /rst/figures/demos/workload_distribution_basic_demo.png
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

    colcon build --packages-up-to amlip_demo_nodes

Once AML-IP packages are installed and built, import the libraries using the following command.

.. code-block:: bash

    source install/setup.bash

Explaining the demo
===================

In this section, we will delve into the details of the demo and how it works.

Main Node
---------

This node simulates a :ref:`user_manual_nodes_main`.
It does not use real *AML Jobs*, but strings.
It is implemented in |python| using :code:`amlip_py` API.

The code can be found `here <https://github.com/eProsima/AML-IP/blob/main/amlip_demo_nodes/amlip_workload_distribution_demo/main_node_sync.py>`__.

Computing Node
--------------

This node simulates a :ref:`user_manual_nodes_computing`.
It does not use real *AML Jobs*, but strings.
It does not have a real *AML Engine* but instead the calculation is an *upper-case* conversion of the string received.
It is implemented in |cpp| using :code:`amlip_cpp` API.

The code can be found `here <https://github.com/eProsima/AML-IP/blob/main/amlip_demo_nodes/amlip_workload_distribution_demo/computing_node_sync.cpp>`__.

Running the demo
================

This demo runs the implemented nodes in ``amlip_demo_nodes/amlip_workload_distribution_demo``.

Run Main Node
-------------

Take into account that this node will wait until there are *Computing Nodes* running and available
in the same :term:`LAN` in order to solve the jobs.

There are 2 different ways to run it, an automatic one and a manual one:

Automatic version
^^^^^^^^^^^^^^^^^

In this version, the python executable expects input arguments.
For each argument, it will convert it to a string (:code:`str`) and send it as a *Job*.
Once the arguments run out, it will finish execution and destroy the Node.

Run the following command:

.. code-block:: bash

    # Source colcon installation
    source install/setup.bash

    # To execute Main Node to send 2 jobs
    cd install/amlip_workload_distribution_demo/bin/
    python3 main_node_sync.py first_job "second job"

The expected output is the following:

.. code-block:: bash

    Main Node AMLMainNode.aa.a5.47.fe ready.
    Main Node AMLMainNode.aa.a5.47.fe sending task <first_job>.
    # ... Waits for Computing Node
    Main Node received solution from AMLComputingNode.d1.c3.86.0a for job <first_job> => <FIRST_JOB>.
    Main Node AMLMainNode.aa.a5.47.fe sending task <second job>.
    Main Node received solution from AMLComputingNode.d1.c3.86.0a for job <second job> => <SECOND JOB>.
    Main Node AMLMainNode.aa.a5.47.fe closing.

Manual version
^^^^^^^^^^^^^^

In this version the python program expects to receive keyboard input.
For each keyboard input received, it will convert it to a string (:code:`str`) and send it as a *Job*.
When empty string given, it will finish execution and destroy the Node.

Run the following command:

.. code-block:: bash

    # Source colcon installation
    source install/setup.bash

    # To execute Main Node to send jobs
    cd install/amlip_workload_distribution_demo/bin/
    python3 main_node_sync.py

The expected output is the following:

.. code-block:: bash

    Main Node AMLMainNode.aa.a5.47.fe ready.
    Please enter a string to create a job. Press enter to finish:
    > first_job
    Main Node AMLMainNode.aa.a5.47.fe sending task <first_job>.
    # ... Waits for Computing Node
    Main Node received solution from AMLComputingNode.d1.c3.86.0a for job <first_job> => <FIRST_JOB>.
    Please enter a string to create a job. Press enter to finish:
    > second job
    Main Node AMLMainNode.aa.a5.47.fe sending task <second job>.
    # ... Waits for Computing Node
    Main Node received solution from AMLComputingNode.d1.c3.86.0a for job <second job> => <SECOND JOB>.
    Please enter a string to create a job. Press enter to finish:
    >
    Main Node AMLMainNode.aa.a5.47.fe closing.

Run Computing Node
------------------

To run it, one integer argument is required.
This will be the number of jobs this Node will answer to before finishing its execution and being destroyed.

Take into account that this node will wait until it has solved 2 different jobs.
If there are more than 1 *Computing Node* running, one job is only solved by one of them.

Run the following command to answer 2 jobs before closing:

.. code-block:: bash

    # Source colcon installation
    source install/setup.bash

    # To execute Computing Node to answer 2 jobs
    cd install/amlip_workload_distribution_demo/bin/
    ./computing_node_sync 2

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
