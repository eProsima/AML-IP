.. include:: ../../exports/alias.include

.. |status| replace:: *Status*

.. _user_manual_scenarios_status:

##############################
Monitor Network State Scenario
##############################

This :term:`Scenario` performs the monitoring action: knowing, analyzing and debugging an |aml| network.
Each of the |amlip| :term:`nodes <Node>` :term:`publishes <Publish>` its current |status| information and updates it along its lifetime.
This scenario supports :term:`subscription <Subscribe>` to this :term:`Topic` in order to receive such status information, that can be processed, stored, read, etc.

.. figure:: /rst/figures/scenarios/status_scenario.png
    :align: center
    :width: 80%

.. _user_manual_status_data_type:

Status Data Type
================

The |status| published by the nodes has the following information:

* **Node Id**: Uniquely identifies a Node. Check following :ref:`section <user_manual_scenarios_nodes_id>`.
* **Current State**: The current state of the node, that can be :code:`stopped`, :code:`running` or :code:`dropped`.
* **Node Kind**: Specifies which kind of node is, and so to which Scenario belongs.

Nodes
=====

This scenario involves every Node, as all of them publish the |status| information.
However, the only Node Kind properly belonging to this Scenario is :ref:`user_manual_nodes_status`.
