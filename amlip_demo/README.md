# eProsima AML-IP Demo Nodes

This is a package with a demo on AML-IP node execution.
In this package there are 2 executables, `main` and `computing`.

## Demo

This intends to recreate a simple real scenario where a user with a C++ based *Computing Node* waits and answers
tasks from a Python implementation of a *Main Node*.
These nodes can be run from anywhere in the LAN, and as much of each kind as desired, and the workload
of each *Main* will be distributed along the *Computing* available.

This project intends to simulate what a real future AML network would look like,
as well as to give a simple example on how to implement, use and deploy AML-IP nodes.

<img src="basic_example.png" width="300"/>

## AML Engine Mock

The computation is not based in AML per-se, but in a mock of the engine.
Instead of the Engine and the actual data types used in AML, it sends a `string` as a job, and the solution of this job is this same string in upper case.
However, to make the user experience more realistic, the calculation of this to-upper-case is done with a random delay time between 1 and 5 seconds.

---

## Nodes simulated

### Main Node

This is a **Python** application that sends jobs in the format of strings.
From each string given it creates a job, send it and awaits for a computing to receive the solution.
There are 2 ways to introduce strings to send jobs:

1. Each argument to the python command will be converted to a string and sent as a job.
2. If no arguments are given, it expects user to enter strings by keyboard input. Entering an empty string will close the execution.

```sh
# To send 2 jobs
python3 ./install/amlip_demo_nodes/bin/main_node.py first_job "second job"

# To waits for keyboard input
python3 ./install/amlip_demo_nodes/bin/main_node.py
```

Its implementation can be seen in `main_node.py` file.

### Computing Node

This is a **C++** application that waits for a job to arrive from a Main node and returns the solution.
It receives an integer argument to set the number of tasks that this node must wait before closing.

```sh
# To answer 2 jobs
./install/amlip_demo_nodes/bin/computing_node 2
```

Its implementation can be seen in `computing_node.cpp` file.

---

## Build dependencies

```sh
mkdir src
wget https://raw.githubusercontent.com/eProsima/AML-IP/feature/aml-demos/amlip.repos
vcs import src < amlip.repos
colcon build --packages-up-to amlip_demo_nodes
source install/setup.bash
```

### Execute nodes

```sh
# To execute Main Node to send 2 jobs
python3 ./install/amlip_demo_nodes/bin/main_node.py first_job "second job"

# To execute Computing Node to answer 2 jobs
./install/amlip_demo_nodes/bin/computing_node 2
```
