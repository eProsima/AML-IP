.. include:: ../../exports/alias.include

##################
Internal Protocols
##################

This page briefly describes the protocols developed on top of DDS that enable AML-IP node communications as well as the deployment of the different :ref:`scenarios <user_manual_scenarios>` presented in the user manual of this documentation.

DDS Topics
----------

All the |amlip| Topics in the DDS |amlip| network have a previous name mangling.
So an |amlip| topic named , :code:`some_topic` would actually be named :code:`amlip::some_topic` in the underneath DDS network.

.. _protocols_dds_multiservice:

MultiService over DDS
----------------------

A new communication protocol based on DDS has been designed in order to fulfill the necessity of distributing a task in a network.
The idea is a Service protocol based on Client-Server communication where multiple servers could be available at the same time in the same network.
This protocol creates an auto-regulated orchestration method where a task could be distributed to **one and only one** server that is publicly available, and each server receives no more than one task at a time.

.. _protocols_dds_rpc:

RPC over DDS
------------

The Remote Procedure Call based on DDS has been implemented to meet the need to distribute requests and replies across a network.

Unlike how it is implemented in ROS 2 (with Fast DDS as middleware), topic mangling is used for the communication between the servers and clients.

The following diagram illustrates the flow of the implementation:

.. figure:: /rst/figures/rpc_diagram.png
    :align: center
    :width: 100%
