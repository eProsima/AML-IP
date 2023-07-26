.. include:: ../../exports/alias.include

.. _user_manual_nodes_agent:

##########
Agent Node
##########

The Agent Node relays on the `eProsima DDS Router <https://eprosima-dds-router.readthedocs.io/en/latest/index.html>`__.
This tool is developed and maintained by `eProsima` which enables the connection of distributed DDS networks.
DDS entities such as publishers and subscribers deployed in one geographic location and using a dedicated local network will be able to communicate with other DDS entities deployed in different geographic areas on their own dedicated local networks as if they were all on the same network.

This node is in charge of communicating a local node or AML-IP cluster with the rest of the network in WANs.
It centralizes the WAN discovery and communication, i.e. it is the bridge for all the nodes in their LANs with the rest of the AML-IP components.

.. figure:: /rst/figures/agent_nodes.png
    :align: center

.. _user_manual_nodes_agent_client:

***********
Client Node
***********

This node acts as a communication client that connects to a Server Node.

.. figure:: /rst/figures/agent_nodes_client.png
    :align: center
    :width: 75%

.. _user_manual_nodes_agent_server:

***********
Server Node
***********

This node acts as a communication server, waiting for other Client Nodes to connect to it.

.. figure:: /rst/figures/agent_nodes_server.png
    :align: center
    :width: 75%

.. _user_manual_nodes_agent_turn:

*************
Repeater Node
*************

A Repeater Node could work as a :term:`TURN` Repeater.
This means that a Repeater Node can be used to repeat messages between networks.

.. figure:: /rst/figures/agent_nodes_repeater.png
    :align: center
    :width: 75%
