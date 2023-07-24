.. include:: ../../exports/alias.include

.. _user_manual_dds:

###
DDS
###

:term:`DDS` is a distributed dynamic real-time middleware protocol based on a specification defined by the :term:`OMG`.
It relies on the underlying :term:`RTPS` wire protocol.

|amlip| framework relies on :term:`DDS` communication protocol to connect and communicate each of its
:term:`Nodes <Node>`.
:term:`DDS` protocol support :term:`publications <Publish>` and
:term:`subscriptions <Subscribe>` in different :term:`Topics <Topic>` in order to create a distributed network
of entities where communication takes place peer-to-peer, avoiding centralized systems and creating an
homogeneous and stand-alone network.
:term:`DDS` relies on :term:`QoS` to configure different characteristics for each of the communication channels,
allowing to create really dynamic and complex networks.

Fast DDS
========

|amlip| uses |efastdds|, a C++ open-source library that implements :term:`DDS` specification.
|efastdds| has all the features and characteristics needed to power |amlip| communications.
A whole documentation for the |fastdds| project can be found here: |FastDDSDocs|.


Topics
------

All the |amlip| Topics in the DDS |amlip| network have a previous name mangling.
So an |amlip| topic named , :code:`some_topic` would actually be named :code:`amlip::some_topic` in the underneath
DDS network.


.. _user_manual_dds_multiservice:

MultiService
------------

A new communication protocol based on DDS has been designed in order to fulfill the necessity of distributing
a task in a network.
The idea is a Service protocol based on Client-Server communication where multiple servers could be available at
the same time in the same network.
This protocol creates an auto-regulated orchestration method where a task could be distributed to **one and only one**
server that is publicly available, and each server receives no more than one task at a time.

.. _user_manual_dds_rpc:

RPC
---

The Remote Procedure Call based on DDS has been implemented to meet the need to distribute requests and replies across a network.

Unlike how it is implemented in ROS2 (Fast DDS middleware), it is used topic mangling for the communication between the servers and the clients.

The following diagram illustrates the flow of the implementation:

.. figure:: /rst/figures/rpc_diagram.png
    :align: center
    :width: 100%
