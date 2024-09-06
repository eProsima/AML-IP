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
Each Node is associated with a single :term:`Scenario`.

There are various methods to operate or interact with a Node.
Some are standalone applications capable of executing predefined actions,
while others necessitate user interaction to specify the actions a Node should perform based on its status and received data.
In this last case, the Nodes are programming *Objects* that can be instantiated and customized regarding the
action that must be performed.

Nodes attributes
================

.. _user_manual_scenarios_nodes_id:

Node Id
-------

Each Node in the network has a unique identifier.
This identifier is created by combining its *name*, that is given to each node once it is created, and a randomly generated number.
Node names must adhere to the character limitations of less than 28 characters between :code:`0-9 a-z A-Z _ .`

*Examples of Nodes Ids are:*
:code:`AMLIPNode.d2.24.9f.34`,
:code:`My_Custom_Node.58.cd.72.85`,
:code:`status.node_7.37.18.f7.05`.

.. _user_manual_nodes_state:

Node State
----------

The state of a node reflects its current operational status.
Node states can be used to control the actions that the node performs to communicate with other nodes, or to indicate the status of the node to the user.

Nodes modify their states using the method :code:`change_status_(const eprosima::amlip::types::StateKind& new_state)`.

Users can monitor node states by leveraging a :code:`StatusNode` configured to intercept status messages.
This facilitates real-time observation of node activities and enables users to stay informed about the network's operational status.

A Node's state is represented by the :code:`eprosima::amlip::types::StateKind` enumeration that can be one of the following values:

- :code:`unknown`: The node has not defined state kind.
- :code:`running`: The node is running (executing some action).
- :code:`stopped`: The node is stopped.
- :code:`dropped`: The node has been removed from the network.

.. _user_manual_nodes_kind:

Node Kind
---------

Every Node belongs to a specific *Kind*, which determines its behavior and actions.
Nodes are not restricted in the number of instances per kind within a network.
The kind of the nodes follows the Object Oriented Programming ideas, where every Node Kind represents a *class* and can
inherit from other Node *classes* (e.g. every Node Kind inherits from :code:`ParentNode`);
and each running node in the network is an *instance* of the *class* of its Kind.


.. _user_manual_nodes_kinds:

Node Kinds
==========

.. toctree::
    :maxdepth: 1

    agent
    computing
    edge
    fiware
    inference
    main
    model_manager_receiver
    model_manager_sender
    status
