.. include:: ../exports/alias.include

.. _project_overview:

################
Project Overview
################

|eamlip| is a communications framework in charge of data exchange between :term:`AML` nodes
through local or remote networks.
It is designed to allow non-expert users to create and manage a cluster of AML nodes to exploit the distributed and concurrent learning capabilities of AML.
Thus, AML-IP is a communication framework that abstracts the transport protocols from the user, creating a platform that communicates each node without requiring the user to be concerned about communication issues.
It also allows the creation of complex distributed networks with one or multiple users working on the same problem.

AML
===

:term:`AML` is a cutting edge :term:`ML` technology based on algebraic representations of data.
Unlike statistical learning,
|aml| algorithms are robust regarding the statistical properties of the data and are parameter-free.
This makes |aml| a great candidate in the future of ML,
as it is far less sensitive to statistical characteristics of the training data,
and can integrate unstructured and complex abstract information apart from the training data.

|aml| algorithm has several characteristics that makes it a great player in distributed learning.
First, |aml| can be trained in parallel from different remote machines,
and can merge the training information without losing information.
It can also be shared and merged with other already trained models and share their learnt information without
revealing the training data-set.


AML-IP
======

|eamlip| is a framework based on different libraries and graphical and non-graphical tools that allow
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


Usage
=====

|amlip| is a complex framework composed of different tools that run independently and out-of-the-box.
But it also features some libraries that allow to instantiate |amlip| entities or :term:`Nodes <Node>` whose behavior and functionality must be specified by the user.
These libraries are presented in 2 main programming languages:

C++
---

This is the main programming language in |amlip|.
|cpp| has been chosen because it is a very versatile and complete language that allows to easily implement complex concepts maintaining high performance.
Also |fastdds| is mainly built in |cpp| and using the same programming language allows to easily interact without losing performance with the middleware layer.

There is a public :term:`API` found in :code:`AML-IP/amlip_cpp/include` with all the installed headers that can be used from the user side.
The API, implementation and testing of this part of the code can be found mainly under sub-package :code:`amlip_cpp`.

Python
------

This is the programming language thought to be used by a final user.
|python| has been chosen as it is easier to work with state-of-the-art :term:`ML` projects.

Nodes and classes that the user needs to instantiate in order to implement their own code are parsed from |cpp| by using |swig| tool, giving the user a |python| API.
The API, implementation and testing of this part of the code can be found mainly under sub-package :code:`amlip_py`.


Architecture and Infrastructure
===============================

|amlip| is a software project based on different programming languages.
It is a **public** **open-source** project focused to be used by the :term:`ML` and scientific community.
The whole project is hosted on a |github| repository, and can be found in the following url: |githubpage|.
The code project is divided in sub-packages that can be built, installed and tested independently.

|amlip| is a software project that does not rely on any specific hardware or Operating System, and does not require any physical infrastructure.
The storage and CI is hosted by |github|.


Testing and CI
==============

|amlip| project is exhaustively tested with unit, integration, manual and blackbox tests that can be found within the source code.
As an open-source project, every person can and is welcome to contribute to the project by implementing, fixing, suggesting or issuing.
Every new contribution to the project must be peer reviewed by a project member before being accepted and merged, and must fulfill all the tests and required :term:`CI`.
