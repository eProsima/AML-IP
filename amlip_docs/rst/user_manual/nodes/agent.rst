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

Steps
-----

* Create a new :code:`eprosima::ddspipe::participants::types::Address` object with the address port, external address port, :term:`IP` address and transport protocol.
* Instantiate the Client Node creating an object of such class with a name and the connection address.

.. tabs::

    .. tab:: C++

        .. code-block:: cpp

            // Create connection address
            auto connection_address = eprosima::ddspipe::participants::types::Address(
                12121,
                12121,
                "localhost",
                eprosima::ddspipe::participants::types::TransportProtocol::udp);

            // Create Client Node
            eprosima::amlip::node::agent::ClientNode Client_node(
                "CppClientNode_Manual",
                { connection_address });

            // Wait until Ctrl+C
            eprosima::utils::event::SignalEventHandler<eprosima::utils::event::Signal::sigint> sigint_handler;
            sigint_handler.wait_for_event();


.. _user_manual_nodes_agent_server:

***********
Server Node
***********

This node acts as a communication server, waiting for other Client Nodes to connect to it.

.. figure:: /rst/figures/agent_nodes_server.png
    :align: center
    :width: 75%

Steps
-----

* Create a new :code:`eprosima::ddspipe::participants::types::Address` object with the address port, external address port, :term:`IP` address and transport protocol.
* Instantiate the Server Node creating an object of such class with a name and the listening address.

.. tabs::

    .. tab:: C++

        .. code-block:: cpp

            // Create listening address
            auto listening_address = eprosima::ddspipe::participants::types::Address(
                12121,
                12121,
                "localhost",
                eprosima::ddspipe::participants::types::TransportProtocol::udp);

            // Create Server Node
            eprosima::amlip::node::agent::ServerNode Client_node(
                "CppServerNode_Manual",
                { listening_address });

            // Wait until Ctrl+C
            eprosima::utils::event::SignalEventHandler<eprosima::utils::event::Signal::sigint> sigint_handler;
            sigint_handler.wait_for_event();


.. _user_manual_nodes_agent_repeater:

*************
Repeater Node
*************

A Repeater Node can be used to repeat messages between networks, that is, the message will be forwarded using the same network interface. This is useful to communicate across LANs.

.. figure:: /rst/figures/agent_nodes_repeater.png
    :align: center
    :width: 75%

Steps
-----

* Create a new :code:`eprosima::ddspipe::participants::types::Address` object with the address port, external address port, :term:`IP` address and transport protocol.
* Instantiate the Client Node creating an object of such class with a name and the listening address.

.. tabs::

    .. tab:: C++

        .. code-block:: cpp

            // Create listening address
            auto listening_address = eprosima::ddspipe::participants::types::Address(
                12121,
                12121,
                "localhost",
                eprosima::ddspipe::participants::types::TransportProtocol::udp);

            // Create Repeater Node
            eprosima::amlip::node::agent::RepeaterNode repeater_node(
                "CppRepeaterNode_Manual",
                { listening_address });

            // Wait until Ctrl+C
            eprosima::utils::event::SignalEventHandler<eprosima::utils::event::Signal::sigint> sigint_handler;
            sigint_handler.wait_for_event();
