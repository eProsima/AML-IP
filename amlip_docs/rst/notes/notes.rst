.. include:: ../exports/alias.include

.. _release_notes:

.. comment the include of forthcoming when new info is added

..
    .. include:: forthcoming_version.rst

##############
Version v0.1.0
##############

This is the first release of eProsima *AML-IP* (Algebraic Machine Learning - Integrating Platform).

This release includes the following **User Interface features**:

* C++ API
* Python API
* Add implementation of DDS entities.
* Add implementation of Multiservice protocol.
* Add implementation of Asynchronous Multiservice protocol.
* New internal package :code:`amlip_demo_nodes` to include the demos packages.
* Dockerfile for creating a docker image with AML-IP.
* Add Custom RPC communication over DDS.

This release supports the following **Deployment Scenarios**:

* :ref:`Monitor State <user_manual_scenarios_status>`
* :ref:`Workload Distribution <user_manual_scenarios_workload_distribution>`
* :ref:`Collaborative Learning <user_manual_scenarios_collaborative_learning>`
* :ref:`Distributed Inference <user_manual_scenarios_distributed_inference>`

This release includes the following new **AML-IP Nodes**:

* :ref:`Status <user_manual_nodes_status>`: node that listens to other nodes status.
* :ref:`Main <user_manual_nodes_main>`: node that sends training data and collects the solution to that data.
* :ref:`Computing <user_manual_nodes_computing>`: node that waits for training data and retrieves a solution.
* :ref:`Edge <user_manual_nodes_edge>`: node that sends data and waits for the inferred solution.
* :ref:`Inference <user_manual_nodes_inference>`: node that waits for data and retrieves an inference.
* :ref:`Agent <user_manual_nodes_agent>`: node in charge of the communication with the network.
    * :ref:`Client <user_manual_nodes_agent_client>`
    * :ref:`Server <user_manual_nodes_agent_server>`
    * :ref:`Repeater <user_manual_nodes_agent_repeater>`
* :ref:`Model Manager Receiver <user_manual_nodes_model_receiver>`: node that receives statistical data from models and sends requests to those models.
* :ref:`Model Manager Sender <user_manual_nodes_model_sender>`: node that sends statistical data from models, receives requests and sends replies to those requests.

This release includes the following new **AML-IP Data Types**:

* :ref:`Status <user_manual_status_data_type>`: status messages sent by each node with its id, type and current state.
* :ref:`Job <user_manual_scenarios_workload_distribution_job>`: messages that represent training data.
* :ref:`Job Solution <user_manual_scenarios_workload_distribution_solution>`: messages that represent the solution for a given set of training data.
* :ref:`Inference <user_manual_datatype_inference>`: messages that represent a partial data-set.
* :ref:`Inference Solution <user_manual_datatype_inference_solution>`: messages that represent the inference of a data-set.
* :ref:`Model Statistics <user_manual_scenarios_collaborative_learning_statistics>`: messages that represent statistical data from models.
* :ref:`Model <user_manual_scenarios_collaborative_learning_model>`: messages that represent a problem model request.
* :ref:`Model Solution <user_manual_scenarios_collaborative_learning_solution>`: messages that represent a problem reply with the requested model.

This release includes the following **Demos**:

* :ref:`demo_collaborative_learning`
* :ref:`demo_inference`
* :ref:`demo_rosbot2r_inference`
* :ref:`demo_workload_distribution`

This release includes the following **Documentation features**:

* This same Documentation
* API Code Documentation

This release includes the following **Continuous Integration features**:

* Continuous Integration deployment in `GitHub Actions <https://github.com/eProsima/AML-IP/actions>`_.
* Compile with *-Wall* flag *Clang* job.
* Add *Address Sanitizer* check to all tests.
* Add Python Liner test to the Python API.
* Disable Data Sharing from test.
