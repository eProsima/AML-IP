.. include:: ../../exports/alias.include

#####################
Enabling technologies
#####################

This page describes the different technologies that support the development of the |amlip|.
These focus on the communication between nodes, the protocols used to support such communication and the libraries and tools used to handle the different types of data to be transmitted.

.. _technologies_dds:

DDS (Data Distribution Service)
===============================

:term:`DDS` is a distributed dynamic real-time middleware protocol based on a specification defined by the :term:`OMG`.
It relies on the underlying :term:`RTPS` wire protocol.

|amlip| framework relies on :term:`DDS` communication protocol to connect and communicate each of its :term:`Nodes <Node>`.
:term:`DDS` protocol support :term:`publications <Publish>` and :term:`subscriptions <Subscribe>` in different :term:`Topics <Topic>` in order to create a distributed network of entities where communication takes place peer-to-peer, avoiding centralized systems and creating an homogeneous and stand-alone network.
:term:`DDS` relies on :term:`QoS` to configure different characteristics for each of the communication channels, allowing to create really dynamic and complex networks.

Fast DDS
--------

|amlip| uses |efastdds|, a C++ open-source library that implements :term:`DDS` specification.
|efastdds| has all the features and characteristics needed to power |amlip| communications.
A whole documentation for the |fastdds| project can be found in |FastDDSDocs|.