# eProsima AML-IP Demo

This is a package with a demo on AML-IP node execution.
In this package there are 2 executables, `main` and `computing`.

## AML Engine Mock

This demo mocks the AML Engine.
Instead of the Engine and the actual data types used in AML, it sends as a job a string, and the solution of this job is the string in upper case.

## Main Node

This is a **Python** application that sends jobs in the format of strings.
From each string given it creates a job, send it and awaits for a computing to receive the solution.
There are 2 ways to introduce strings to send jobs:
1. Each argument to the python command will be converted to a string and sent as a job.
2. If no arguments are given, it expects user to enter strings by keyboard input. Entering an empty string will close the execution.

```sh
# To send 2 jobs
python3 ./build/amlip_demo/main_node.py first_job "second job"

# To waits for keyboard input
python3 ./build/amlip_demo/main_node.py
```

## Computing Node

This is a **C++** application that waits for a job to arrive from a Main node and returns the solution.
It receives an integer argument to set the number of tasks that this node must wait before closing.

```sh
# To answer 2 jobs
./build/amlip_demo/computing_node 2
```

---

## Build dependencies

```sh
mkdir src
wget https://raw.githubusercontent.com/eProsima/AML-IP/feature/aml-demos/amlip.repos
vcs import src < amlip.repos
colcon build
source install/setup.bash

# To execute Main Node
python3 ./build/amlip_demo/main_node.py first_job "second job"

# To execute Computing Node
./build/amlip_demo/computing_node 2
```
