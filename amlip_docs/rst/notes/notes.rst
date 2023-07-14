.. include:: ../exports/alias.include

.. _release_notes:

.. comment the include of forthcoming when new info is added

..
    .. include:: forthcoming_version.rst

##############
Version v0.1.0
##############

This is the first release of eProsima *AML-IP*.

This release includes the following **User Interface features**:

* C++ API
* Python API
* Add implementation of DDS entities.
* Add implementation of Multiservice protocol.
* Add implementation of Asynchronous Multiservice protocol.
* New internal package :code:`amlip_demo_nodes` to include the demos packages.
* Dockerfile for creating a docker image with AML-IP.

This release supports the following **:term:`Scenarios <Scenario>` features**:

* :ref:`Monitor State <user_manual_scenarios_status>`
* :ref:`Workload Distribution <user_manual_scenarios_workload_distribution>`

This release includes the following **:term:`Nodes <Node>` features**:

* :ref:`Status <user_manual_nodes_status>`
* :ref:`Main <user_manual_nodes_main>`
* :ref:`Computing <user_manual_nodes_computing>`
* Asynchronous Main
* Asynchronous Computing
* :ref:`Edge <user_manual_nodes_edge>`
* :ref:`Inference <user_manual_nodes_inference>`
* :ref:`Asynchronous Edge <user_manual_nodes_async_edge>`
* :ref:`Asynchronous Inference <user_manual_nodes_async_inference>`
* Agent Nodes:
    * Client
    * Server
    * Turn

This release includes the following **Data Types features**:

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

* Compile with *-Wall* flag *Clang* job.
* Add *Address Sanitizer* check to all tests.
* Add Python Liner test to the Python API.
* Disable Data Sharing from test.
