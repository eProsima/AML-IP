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

This release includes the following new **AML-IP Nodes**:

* :ref:`Status <user_manual_nodes_status>`
* :ref:`Main <user_manual_nodes_main>`
* :ref:`Computing <user_manual_nodes_computing>`
* :ref:`Edge <user_manual_nodes_edge>`
* :ref:`Inference <user_manual_nodes_inference>`
* Agent Nodes:
    * Client
    * Server
    * Turn

This release includes the following new **AML-IP Data Types**:

* :ref:`Status <user_manual_status_data_type>`
* :ref:`Job <user_manual_scenarios_workload_distribution_job>`
* :ref:`Job Solution <user_manual_scenarios_workload_distribution_solution>`
* :ref:`Inference <user_manual_datatype_inference>`
* :ref:`Inference Solution <user_manual_datatype_inference_solution>`

This release includes the following **Demos**:

* :ref:`demo_workload_distribution`
* :ref:`demo_inference`
* :ref:`demo_rosbot2r_inference`

This release includes the following **Documentation features**:

* This same Documentation
* API Code Documentation

This release includes the following **Continuous Integration features**:

* Continuous Integration deployment in `GitHub Actions <https://github.com/eProsima/AML-IP/actions>`_.
* Compile with *-Wall* flag *Clang* job.
* Add *Address Sanitizer* check to all tests.
* Add Python Liner test to the Python API.
* Disable Data Sharing from test.
