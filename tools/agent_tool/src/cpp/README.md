# Agent Node tool

*eprosima AML-IP* C++ application to execute an Agent Node.

## Building the workspace

mkdir -p ~/AML-IP/src
cd ~/AML-IP
wget https://raw.githubusercontent.com/eProsima/AML-IP/main/amlip.repos
vcs import src < amlip.repos
colcon build

## Execution instructions

In order to know all the possible arguments supported by this tool, use the command:

```sh
  ./agent tool -h
```

To launch an Agent Server Node, execute:

```sh
  ./agent tool -e server
```

To launch an Agent Repeater Node, execute:

```sh
  ./agent tool -e repeater
```

## Arguments

The arguments are read unordered

```sh
Usage: ./agent tool

Usage: ./agent tool

General options:
  -h, --help
                    Produce help message.
  -e, --entity <client|server|repeater>
                    Agent Entity type (Default: client). Allowed options:

                    • client -> Run an Agent Client Node.

                    • server -> Run an Agent Server Node.

                    • repeater -> Run an Agent Repeater Node.
  -n, --name <name>
                     Name (Default: agent_tool).
  -d, --domain <id>
                    DDS domain ID (Default: 0).
  -p, --connection-port <num>
                    Address connection port (Default: 12121).
  -x, --listening-port <num>
                    Address listening port (Default: 12121).
  -c, --connection <connection_address>
                    Address to connect (Default: 127.0.0.1).
  -l, --listening <listening_address>
                    Address where listen (Default: 127.0.0.1).
  -t, --transport <shm|udp|udpv6>
                    Use only shared-memory, UDPv4, or UDPv6 transport. (Default: udp).

```
