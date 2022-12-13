.. include:: ../../exports/alias.include

.. _user_manual_nodes:

###########
AML-IP Node
###########

An |amlip| network is divided in independent stand-alone individuals named :term:`Nodes <Node>`.
A Node is understand as a software piece that perform one or multiple :term:`Actions <Action>` in a auto-managing
way, thus it does not require external orchestration neither a central point of computation.
These actions can local, such as calculations, data process, algorithm executions, etc.
Or can be communication actions, as send messages, receive data, wait for data or specific status, etc.
Each Node belongs to one and only one :term:`Scenario`.

There are different ways to run or to work with a Node.
Some of them are applications that can be executed and perform a fixed action.
Others, however, require a user interaction as specifying the action such Node must perform depending on its status
and the data received.
In this last case, the Nodes are programming *Classes* that can be instantiated and customize regarding the
action that must be performed.

Nodes attributes
================

.. _user_manual_scenarios_nodes_id:

Node Id
-------

Node Kind
---------

.. warning::

    |amlip| is a work in progress, and these interactions and usage can change along the process.


Node Kinds
==========

.. toctree::
    :maxdepth: 1

    status
    main
    computing
