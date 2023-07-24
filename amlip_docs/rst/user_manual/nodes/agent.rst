.. include:: ../../exports/alias.include

.. _user_manual_nodes_agent:

##########
Agent Node
##########

The Agent Node will relay on the DDS Router.
This tool is developed and maintained by `eProsima` which facilitates certain DDS behaviours, as the WAN or the **UPnP** (Universal Plug and Play) administration.
It allows to create a bridge between two DDS DomainParticipants, which fill the gap between the LAN and WAN.

This node is in charge of communicating a local node or AML-IP cluster with the rest of the network in WANs.
It centralizes the WAN discovery and communication, i.e. it is the bridge for all the nodes in their LANs with the rest of the AML-IP components.

Does not depend on any AML node, so the AML process is not affected by any communication delays.

.. figure:: /rst/figures/agent_nodes.png
    :align: center
    :width: 75%

.. _user_manual_nodes_agent_client:

***********
Client Node
***********

This node acts as a communication client that connects to a Server Node.

.. figure:: /rst/figures/agent_nodes_client.png
    :align: center

.. _user_manual_nodes_agent_server:

***********
Server Node
***********

This node acts as a communication server, waiting for other Client Nodes to connect to it.

.. figure:: /rst/figures/agent_nodes_server.png
    :align: center
    :width: 75%

.. _user_manual_nodes_agent_turn:

*********
Turn Node
*********

A Turn Node could work as a :term:`TURN` Repeater.
This means that a Turn Node can be used to repeat messages between networks.

.. figure:: /rst/figures/agent_nodes_turn.png
    :align: center
    :width: 75%
