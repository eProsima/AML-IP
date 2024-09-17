.. include:: ../../exports/alias.include

.. _user_manual_nodes_agent:

##########
Agent Node
##########

The Agent Node relays on the `eProsima DDS Router <https://eprosima-dds-router.readthedocs.io/en/latest/index.html>`__.
This tool is developed and maintained by `eProsima` which enables the connection of distributed DDS networks.
DDS entities such as publishers and subscribers deployed in one geographic location and using a dedicated local network will be able to communicate with other DDS entities deployed in different geographic areas on their own dedicated local networks as if they were all on the same network.

This node is in charge of communicating a local node or AML-IP cluster with the rest of the network in WANs.
It serves as the central hub for WAN discovery and communication, acting as a bridge that connects all nodes within their respective LANs to the broader AML-IP network.

.. figure:: /rst/figures/agent_nodes.png
    :align: center

.. _user_manual_nodes_agent_client:

***********
Client Node
***********

This node acts as a communication client that connects to a Server Node.

.. figure:: /rst/figures/agent_nodes_client.png
    :align: center
    :width: 80%

Steps
-----

* Create a new :code:`eprosima::ddspipe::participants::types::Address` object with the address port, external address port, :term:`IP` address and transport protocol.
* Instantiate the ``ClientNode`` creating an object of this class with a name, a connection address and a domain.
* Wait until ``Ctrl+C``.

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
                { connection_address },
                100);

            // Wait until Ctrl+C
            eprosima::utils::event::SignalEventHandler<eprosima::utils::event::Signal::sigint> sigint_handler;
            sigint_handler.wait_for_event();

    .. tab:: Python

        .. code-block:: python

            # Create connection address
            connection_address = Address(
                port=12121,
                external_port=12121,
                domain='localhost',
                transport_protocol=TransportProtocol_udp)

            # Create Client Node
            ClientNode(
                name='PyTestClientNode',
                connection_addresses=[connection_address],
                domain=100)

            # Wait until Ctrl+C
            def handler(signum, frame):
                pass
            signal.signal(signal.SIGINT, handler)
            signal.pause()

.. _user_manual_nodes_agent_server:

***********
Server Node
***********

This node acts as a communication server, waiting for other Client Nodes to connect to it.

.. figure:: /rst/figures/agent_nodes_server.png
    :align: center
    :width: 80%

Steps
-----

* Create a new :code:`eprosima::ddspipe::participants::types::Address` object with the address port, external address port, :term:`IP` address and transport protocol.
* Instantiate the ``ServerNode`` creating an object of this class with a name, a listening address and a domain.
* Wait until ``Ctrl+C``.

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
            eprosima::amlip::node::agent::ServerNode Server_node(
                "CppServerNode_Manual",
                { listening_address },
                200);

            // Wait until Ctrl+C
            eprosima::utils::event::SignalEventHandler<eprosima::utils::event::Signal::sigint> sigint_handler;
            sigint_handler.wait_for_event();

    .. tab:: Python

        .. code-block:: python

            # Create listening address
            listening_address = Address(
                port=12121,
                external_port=12121,
                domain='localhost',
                transport_protocol=TransportProtocol_udp)

            # Create Server Node
            ServerNode(
                name='PyTestServerNode',
                listening_addresses=[listening_address],
                domain=200)

            # Wait until Ctrl+C
            def handler(signum, frame):
                pass
            signal.signal(signal.SIGINT, handler)
            signal.pause()

.. _user_manual_nodes_agent_repeater:

*************
Repeater Node
*************

A Repeater Node is utilized to forward messages between different networks, effectively repeating the message using the same network interface. This functionality is particularly useful for facilitating communication across multiple LANs.

.. figure:: /rst/figures/agent_nodes_repeater.png
    :align: center
    :width: 80%

Steps
-----

* Create a new :code:`eprosima::ddspipe::participants::types::Address` object with the address port, external address port, :term:`IP` address and transport protocol.
* Instantiate the ``RepeaterNode`` creating an object of this class with a name, a listening address and a domain.
* Wait until ``Ctrl+C``.

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

    .. tab:: Python

        .. code-block:: python

            # Create listening address
            listening_address = Address(
                port=12121,
                external_port=12121,
                domain='localhost',
                transport_protocol=TransportProtocol_udp)

            # Create Repeater Node
            RepeaterNode(
                name='PyTestRepeaterNode',
                listening_addresses=[listening_address])

            # Wait until Ctrl+C
            def handler(signum, frame):
                pass
            signal.signal(signal.SIGINT, handler)
            signal.pause()
