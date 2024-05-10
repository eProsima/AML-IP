.. include:: ../../exports/alias.include

.. _user_manual_nodes:

###########
AML-IP Node
###########

An |amlip| network is divided in independent stand-alone individuals named :term:`Nodes <Node>`.
A Node, understood as a software piece that performs one or multiple :term:`Actions <Action>` in a auto-managing way,
does not require external orchestration neither a central point of computation.
These actions can be local actions such as calculations, data process, algorithm executions, etc.,
or communication actions as send messages, receive data, wait for data or specific status, etc.
Each Node belongs to one and only one :term:`Scenario`.

There are different ways to run or to work with a Node.
Some of them are applications that can be executed and perform a fixed action.
Others, however, require a user interaction as specifying the action such Node must perform depending on its status
and the data received.
In this last case, the Nodes are programming *Objects* that can be instantiated and customized regarding the
action that must be performed.

Nodes attributes
================

.. _user_manual_scenarios_nodes_id:

Node Id
-------

Each Node in a network has a unique Id.
This Id is created by the Node *name*, that is given to each node once it is created, and a randomly generated number.
The name of a node must consist of less than 28 characters between :code:`0-9 a-z A-Z _ .`

*Examples of Nodes Ids are:*
:code:`AMLIPNode.d2.24.9f.34`,
:code:`My_Custom_Node.58.cd.72.85`,
:code:`status.node_7.37.18.f7.05`.


Node Kind
---------

Each Node belongs to a specific *Kind*.
The kind of the node identifies it, and makes it behave and perform different actions.
There are no restrictions in the number of nodes of the same kind running in the same network.
The kind of the nodes follows the Object Oriented Programming ideas, where every Node Kind represents a *class* and can
inherit from other Node *classes* (e.g. every Node Kind inherits from :code:`ParentNode`);
and each running node in the network is an *instance* of the *class* of its Kind.


.. _user_manual_nodes_kinds:

Node Kinds
==========

.. toctree::
    :maxdepth: 1

    agent
    status
    main
    computing
    edge
    inference
    model_manager_receiver
    model_manager_sender
