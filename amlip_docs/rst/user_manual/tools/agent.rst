.. include:: ../../exports/alias.include

.. _user_manual_tools_agent:

##########
Agent Tool
##########

This tool launches an :ref:`user_manual_nodes_agent`, which is the main transport component that manages the communication of *AML-IP Algebraic Nodes* deployed within a :term:`LAN` and the discovery of other AML-IP clusters over :term:`WAN`.

Building the tool
=================

If the tool package is not compiled, please refer to :ref:`developer_manual_installation_sources_linux_colcon` or run the command below.

.. code-block:: bash

    colcon build --packages-up-to amlip_agent

Once AML-IP packages are installed and built, source the workspace using the following command.

.. code-block:: bash

    source install/setup.bash

Application Arguments
=====================

The Agent Tool supports several input arguments:

.. list-table::
    :header-rows: 1

    *   - Command
        - Option
        - Long option
        - Value
        - Default Value

    *   - :ref:`Help <user_manual_tool_agent_help_argument>`
        - ``-h``
        - ``--help``
        -
        -

    *   - Entity
        - ``-e``
        - ``--entity``
        - Agent Entity type.
        - ``client``

    *   - Name
        - ``-n``
        - ``--name``
        - Readable File Path
        - ``amlip_agent``

    *   - DDS Domain
        - ``-d``
        - ``--domain``
        - Configures the :term:`Domain Id`.
        - ``0``

    *   - Connection Address
        - ``-c``
        - ``--connection-address``
        - Address to connect.
        - ``127.0.0.1``

    *   - Connection Port
        - ``-p``
        - ``--connection-port``
        - Address connection port.
        - ``12121``

    *   - Listening Address
        - ``-l``
        - ``--listening-address``
        - Address where listen.
        - ``127.0.0.1``

    *   - Listening Port
        - ``-q``
        - ``--listening-port``
        - Address listening port.
        - ``12121``

    *   - Transport
        - ``-t``
        - ``--transport``
        -  Use only TCPv4 or UDPv4 transport.
        - ``TCPv4``

.. _user_manual_tool_agent_help_argument:

Help Argument
-------------

It shows the usage information of the tool.

.. code-block:: console

    Usage: ./agent tool

    General options:
    -h, --help
                        Produce help message.
    -e, --entity <client|server|repeater>
                        Agent Entity type (Default: client). Allowed options:

                        • client -> Run an Agent Client Node.

                        • server -> Run an Agent Server Node.

                        • repeater -> Run an Agent Repeater Node.

    Client options:
    -n, --name <name>
                        Name (Default: amlip_agent).
    -d, --domain <id>
                        DDS domain ID (Default: 0).
    -c, --connection-address <address>
                        Address to connect (Default: 127.0.0.1).
    -p, --connection-port <num>
                        Address connection port (Default: 12121).
    -t, --transport <tcp|udp>
                        Use only TCPv4 or UDPv4 transport. (Default: TCPv4).

    Server options:
    -n, --name <name>
                        Name (Default: agent_tool).
    -d, --domain <id>
                        DDS domain ID (Default: 0).
    -l, --listening-address <address>
                        Address where listen (Default: 127.0.0.1).
    -q, --listening-port <num>
                        Address listening port (Default: 12121).
    -t, --transport <tcp|udp>
                        Use only TCPv4 or UDPv4 transport. (Default: TCPv4).

    Repeater options:
    -n, --name <name>
                        Name (Default: agent_tool).
    -d, --domain <id>
                        DDS domain ID (Default: 0).
    -c, --connection-address <address>
                        Address to connect (Default: 127.0.0.1).
    -l, --listening-address <address>
                        Address where listen (Default: 127.0.0.1).
    -p, --connection-port <num>
                        Address connection port (Default: 12121).
    -q, --listening-port <num>
                        Address listening port (Default: 12121).
    -t, --transport <tcp|udp>
                        Use only TCPv4 or UDPv4 transport. (Default: TCPv4).


Run tool
========

Source the following file to setup the AML-IP environment:

.. code-block:: bash

    source <path-to-amlip-installation>/install/setup.bash

Launching an :ref:`Agent Client Node <user_manual_nodes_agent_client>` instance is as easy as executing the following command:

.. code-block:: bash

    amlip_agent -e client -c 87.111.115.111 -p 18000 -t tcp

To launch an :ref:`Agent Server Node <user_manual_nodes_agent_server>`, execute:

.. code-block:: bash

    amlip_agent -e server -l 87.111.115.111 -q 18000 -t tcp

To launch an :ref:`Agent Repeater Node <user_manual_nodes_agent_repeater>`, execute:

.. code-block:: bash

    amlip_agent -e repeater -l 87.111.115.111 -q 18000 -t tcp

Close tool
==========

In order to stop the *Agent* tool, press ``Ctrl + C`` in the terminal where the process is running.
