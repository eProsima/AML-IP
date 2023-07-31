# Agent Node tool

*eprosima AML-IP* C++ application to execute an *Agent Node* which is the main transport component that manages the dynamic discovery of *AML-IP Algebraic Nodes* deployed within a LAN and the discovery of other AML clusters over WAN.

## Building the workspace

```sh
  mkdir -p ~/AML-IP/src
  cd ~/AML-IP
  wget https://raw.githubusercontent.com/eProsima/AML-IP/main/amlip.repos
  vcs import src < amlip.repos
  colcon build
```

## Execution instructions

In order to know all the possible arguments supported by this tool, use the command:

```sh
  ./agent tool -h
```

To launch an Agent Server Node, execute:

```sh
  ./agent tool -e server -l 87.216.115.84 -q 18000 -t tcp
```

To launch an Agent Client Node, execute:

```sh
  ./amlip_agent -e client -c 87.216.115.84 -p 18000 -t tcp
```

## Arguments

The arguments are read unordered:

```sh
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
```
