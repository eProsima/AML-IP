.. include:: ../../exports/alias.include

.. _user_manual_tools_agent:

##########
Agent Tool
##########

This tool launches an :ref:`user_manual_nodes_agent`, which is the node in charge of communicating a local node or AML-IP cluster with the rest of the network in :term:`WAN`\s.
It centralizes the :term:`WAN` discovery and communication, i.e. it is the bridge for all the nodes in their :term:`LAN`\s with the rest of the AML-IP components.

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

    *   - Debug
        - ``-d``
        - ``--debug``
        - Enables the |amlip| |br|
          logs so the execution can be |br|
          followed by internal |br|
          debugging information. |br|
          Sets ``Log Verbosity`` |br|
          to ``info`` and |br|
          ``Log Filter`` |br|
          to ``AMLIP``.
        -

    *   - Log Verbosity
        - Set the verbosity level so |br|
          only log messages with equal |br|
          or higher importance level |br|
          are shown.
        -
        - ``--log-verbosity``
        - ``warning``

    *   - Log Filter
        - Set a regex string as filter.
        -
        - ``--log-filter``
        - ``AMLIP``

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
    -h, --help                            Produce help message.
    -e, --entity <client|server|repeater> Agent Entity type (Default: client).
                                            Allowed options:
                                            • client -> Run an Agent Client Node.
                                            • server -> Run an Agent Server Node.
                                            • repeater -> Run an Agent Repeater Node.

    Debug options:
    -d, --debug                           Set log verbosity to Info (Using this option with --log-filter and/or --log-verbosity will head to undefined behaviour).
        --log-filter                      Set a Regex Filter to filter by category the info and warning log entries. (Default = "AMLIP").
        --log-verbosity <info|warning|error>
                                            Set a Log Verbosity Level higher or equal the one given (Default: warning).

    Client options:
    -n, --name <name>                     Name (Default: amlip_agent).
    -d, --domain <id>                     DDS domain ID (Default: 0).
    -c, --connection-address <address>    Address to connect (Default: 127.0.0.1).
    -p, --connection-port <num>           Address connection port (Default: 12121).
    -t, --transport <tcp|udp>             Use only TCPv4 or UDPv4 transport. (Default: TCPv4).

    Server options:
    -n, --name <name>                     Name (Default: amlip_agent).
    -d, --domain <id>                     DDS domain ID (Default: 0).
    -l, --listening-address <address>     Address where listen (Default: 127.0.0.1).
    -q, --listening-port <num>            Address listening port (Default: 12121).
    -t, --transport <tcp|udp>             Use only TCPv4 or UDPv4 transport. (Default: TCPv4).

    Repeater options:
    -n, --name <name>                     Name (Default: amlip_agent).
    -d, --domain <id>                     DDS domain ID (Default: 0).
    -c, --connection-address <address>    Address to connect (Default: 127.0.0.1).
    -l, --listening-address <address>     Address where listen (Default: 127.0.0.1).
    -p, --connection-port <num>           Address connection port (Default: 12121).
    -q, --listening-port <num>            Address listening port (Default: 12121).
    -t, --transport <tcp|udp>             Use only TCPv4 or UDPv4 transport. (Default: TCPv4).


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
