.. raw:: html

  <h1>
    eProsima AML-IP Documentation
  </h1>

.. image:: /rst/figures/logo.png
  :height: 100px
  :width: 100px
  :align: left
  :alt: eProsima
  :target: http://www.eprosima.com/

.. image:: /rst/figures/alma_logo.png
  :height: 100px
  :align: left
  :target: https://alma-ai.eu/

*eProsima AML-IP* is a communications framework in charge of data exchange between Algebraic Machine Learning (AML)
nodes through local or remote networks.
It is designed to allow non-experts users to create and manage a cluster of AML nodes
to exploit the distributed and concurrent learning capabilities of AML.
Thus, AML-IP is a communication framework that makes the transport protocols abstracted from the user,
creating a platform that communicates each node without requiring the user to be concerned about communication issues.
It also allows the creation of complex distributed networks with one or multiple users working on the same problem.

This framework is developed as part of the `ALMA project <https://alma-ai.eu/>`_,
which has received funding from the European Union's Horizon 2020 research and innovation programme
under grant agreement No 952091.
The aim of the EU-funded ALMA project is to leverage AML properties to develop a new generation of interactive,
human-centric machine learning systems.
These systems are expected to reduce bias and prevent discrimination,
remember what they know when they are taught something new,
facilitate trust and reliability and integrate complex ethical constraints into human-artificial intelligence systems.
Furthermore, they are expected to promote distributed, collaborative learning.

.. figure:: /rst/figures/aml_overview.png

########
Overview
########

Following are the main scenarios that the current *AML-IP* supports:

* :ref:`Monitor Network State Scenario <user_manual_scenarios_status>`:
  Analyze the state of a network remotely.
* :ref:`Workload Distribution Scenario <user_manual_scenarios_workload_distribution>`:
  Distribute the machine learning training phase to multiple nodes to parallelize heavy computation.
* :ref:`Collaborative Learning Scenario <user_manual_scenarios_collaborative_learning>`:
  Share models between nodes without having to share the private dataset with which the model was trained.
* :ref:`Distributed Inference Scenario <user_manual_scenarios_distributed_inference>`:
  Distribute large amounts of data to multiple nodes to perform inference in parallel.

Check section :ref:`project_overview` to have a further explanation of the concepts and use cases of this project.

###############################
Contacts and Commercial support
###############################

Find more about us at `eProsima's webpage <https://eprosima.com/>`_.

Support available at:

* Email: support@eprosima.com
* Phone: +34 91 804 34 48

#################################
Contributing to the documentation
#################################

*AML-IP Documentation* is an open source project, and as such all contributions, both in the form of
feedback and content generation, are most welcomed.
To make such contributions, please refer to the
`Contribution Guidelines <https://github.com/eProsima/all-docs/blob/master/CONTRIBUTING.md>`_ hosted in our GitHub
repository.

##############################
Structure of the documentation
##############################

This documentation is organized into the sections below.

* :ref:`Installation Manual <installation_manual_linux>`
* :ref:`Getting Started <project_overview>`
* :ref:`Demo Examples <demo_collaborative_learning>`
* :ref:`User Manual <user_manual_scenarios>`
* :ref:`Developer Manual <developer_manual_installation_sources_linux>`
* :ref:`Release Notes <release_notes>`

----

.. image:: /rst/figures/eu_flag.jpg
  :height: 45px
  :align: left

This project (ALMA: Human Centric Algebraic Machine Learning) has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement No 952091.
