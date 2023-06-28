# eProsima AML-IP Workload Distribution Demo

This is a package containing demos of AML-IP node execution.
In this package there are severat executables:

- [Async Main](main_node_async.py) Async Main Node execution
- [Async Computing](computing_node_async.py) Async Computing Node execution
- [Status](status_node.py) Async Status Node execution
- [Sync Main](main_node_sync.py) Sync Main Node execution
- [Sync Computing](computing_node_sync.py) Sync Computing Node execution

---

## Index

- [eProsima AML-IP Workload Distribution Demo](#eprosima-aml-ip-workload-distribution-demo)
  - [Index](#index)
  - [Demo](#demo)
  - [AML Engine Mock](#aml-engine-mock)
  - [Nodes simulated](#nodes-simulated)
    - [Main Node](#main-node)
    - [Computing Node](#computing-node)
  - [Build Project](#build-project)
  - [Real Use Case](#real-use-case)
  - [Synchronous Nodes simulated](#synchronous-nodes-simulated)
    - [Sync Main Node](#sync-main-node)
    - [Sync Computing Node](#sync-computing-node)

---

## Demo

This intends to recreate a simple real scenario where *Computing Nodes* waits and answers tasks from *Main Nodes*.
These nodes can be run from anywhere in the LAN and as much of each kind as desired.
The workload of each *Main* will be distributed along the *Computing* available.
This is done in a way that even C++ and Python and Sync and Async nodes can work together.

This project intends to simulate what a real future AML network would look like,
as well as to give a simple example on how to implement, use and deploy AML-IP nodes.

<img src="../../.figures/basic_example.png" width="300"/>

---

## AML Engine Mock

The computation is not based in AML per-se, but in a mock of the engine.
Instead of the Engine and the actual data types used in AML, it sends a `string` as a job, and the solution of this job is this same string in upper case.
However, to make the user experience more realistic, the calculation of this to-upper-case is done with a random delay time between 1 and 5 seconds.

---

## Nodes simulated

### Main Node

This is a **Python** application that sends jobs in the format of strings.
From each string given it creates a job and sends it with a configurable time elapsed between job sends.
For each solution received, it prints the solution.
It waits until all jobs have been answered, or `SIGINT / C^ / Ctrl+C` is received.

Its implementation can be seen in [main_node_async.py](main_node_async.py) file.

In order to introduce strings to be sent as jobs, add them as executable inputs.

```sh
# To send 2 jobs
python3 ./install/amlip_demo_nodes/bin/main_node.py FIRST_JOB "Second Job"
```

For further information regarding executable arguments, use `--help` argument.

### Computing Node

This is a **Python** application that waits for a job to arrive from a *Main node* and returns the solution.
It runs until `SIGINT / C^ / Ctrl+C` is received.

Its implementation can be seen in [computing_node_async.py](computing_node_async.py) file.

```sh
# To answer jobs
python3 ./install/amlip_demo_nodes/bin/computing_node.py
```

For further information regarding executable arguments, use `--help` argument.

---

## Build Project

In order to build the following project and be able to operate with this demo Nodes, follow these instructions:

```sh
mkdir src
wget https://raw.githubusercontent.com/eProsima/AML-IP/main/amlip.repos
vcs import src < amlip.repos
colcon build --packages-up-to amlip_demo_nodes
source install/setup.bash
```

---

## Real Use Case

In order to adapt these nodes for a real Use case (substitute the string transformations for real calculations),
look in files for comment `ALGEBRAIC` and substitute with own code.

---

## Synchronous Nodes simulated

There are some demos that contains old examples on Synchronous Nodes.

### Sync Main Node

This is a **Python** application that sends jobs in the format of strings.
From each string given it creates a job, sends it and awaits for a computing to receive the solution.
There are 2 ways to introduce strings to send jobs:

1. Each argument of the Python command is converted to a string and sent as a job.
2. If no arguments are given, it expects user to enter strings by keyboard input.
   Entering an empty string will terminate the execution.

```sh
# To send 2 jobs
python3 ./install/amlip_demo_nodes/bin/main_node.py first_job "second job"

# To waits for keyboard input
python3 ./install/amlip_demo_nodes/bin/main_node.py
```

Its implementation can be seen in [main_node_sync.py](main_node_sync.py) file.

### Sync Computing Node

This is a **C++** application that waits for a job to arrive from a Main node and returns the solution.
It receives an integer argument to set the number of tasks that this node must wait before closing.

```sh
# To answer 2 jobs
./install/amlip_demo_nodes/bin/computing_node 2
```

Its implementation can be seen in [computing_node_sync.cpp](computing_node_sync.cpp) file.
