.. include:: ../exports/alias.include

.. _project_overview:

################
Project Overview
################

|eamlip| is a communications framework in charge of data exchange between :term:`AML` nodes
through local or remote networks.
It is designed to allow non-experts users to create and manage a cluster of AML nodes
to exploit the distributed and concurrent learning capabilities of AML.
Thus, AML-IP is a communication framework that makes the transport protocols abstracted from the user,
creating a platform that communicates each node without requiring the user to be concerned about communication issues.
It also allows the creation of complex distributed networks with one or multiple users working on the same problem.

AML
===

:term:`AML` is a cutting edge :term:`ML` technology based on algebraic representations of data.
Unlike statistical learning,
|aml| algorithms are robust regarding the statistical properties of the data and are parameter-free.
This makes |aml| a great candidate in the future of ML,
as it is far less sensitive to statistical characteristics of the training data,
and can integrate unstructured and complex abstract information apart from the training data.

The |aml| algorithm has several characteristics that makes it a great player in distributed learning.
First, |aml| can be trained in parallel from different remote machines,
and can merge the training information without losing information.
It can also be shared and merged with other already trained models and share their learnt information without
revealing the training data-set.


AML-IP
======

The |eamlip| is a framework based on different libraries and graphical and non-graphical tools that allow
to create a network of nodes focused no different tasks of the |aml| environment.
Every running part of the |amlip| is considered a **Node**.
This is an **independent** and **distributed** software that could perform a specific **action**.

* *Independent* means that it is auto-sufficient and does not require the presence of any other node.
* *Distributed* means that can communicate with different nodes in the network,
  interacting and solving tasks collaboratively.
* *Action* is every part of the |aml| or any satellite action required in order to perform the correct execution of the
  algorithm or to support or facilitate the communication and managing of the different nodes.

These nodes are separated in different scenarios, that are explained more in detail in the
:ref:`following section <user_manual_scenarios>`.

.. figure:: /rst/figures/aml_overview.png
