# eProsima AML-IP Collaborative Learning Demo

This is a package containing demos of AML-IP node execution.
There are several executables in this package, depending on whether they use listeners or callback:

- [Model Receiver Custom](model_receiver_custom.py) Model Manager Receiver Node execution
- [Model Sender Custom](model_sender_custom.py) Model Manager Sender Node execution
- [Model Receiver Lambda](model_receiver_lambda.py) Model Manager Receiver Node execution
- [Model Sender Lambda](model_sender_lambda.py) Model Manager Sender Node execution
- [Model Receiver Listener](model_receiver_listener.py) Model Manager Receiver Node execution
- [Model Sender Listener](model_sender_listener.py) Model Manager Sender Node execution

---

## Index

- [eProsima AML-IP Collaborative Learning Demo](#eprosima-aml-ip-workload-distribution-demo)
  - [Index](#index)
  - [Prepare Workspace](#prepare-workspace)
  - [Prerequisites](#prerequisites)
  - [Build Project](#build-project)
  - [Run demo](#run-demo)
    - [Async Edge Node](#async-edge-node)
    - [Async Inference Node](#async-inference-node)

---

## Prepare Workspace

Create a AML-IP-ws directory and download the repos file that will be used to install AML-IP and its dependencies:

```sh
mkdir ~/AML-IP-ws
cd ~/AML-IP-ws
mkdir src
wget https://raw.githubusercontent.com/eProsima/AML-IP/main/amlip.repos
vcs import src < amlip.repos
```

---

## Build Project

In order to build the project and be able to operate with this demo, follow these instructions:

```sh
colcon build --packages-up-to amlip_collaborative_learning_demo
source install/setup.bash
```

---

## Run demo

Run Model Receiver and Model Sender nodes in two terminals.
In each one, source the workspace and launch a node.


## Nodes simulated

### Model Receiver Node

This is a **Python** application that receives statistical data from models and sends requests to those models depending on the data received.

Its implementation can be seen in [model_receiver_custom.py](model_receiver_custom.py) file.

```sh
# To send request
cd ~/AML-IP-ws/src/AML-IP/amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo
python3 model_receiver_custom.py
```

### Model Sender Node

This is a **Python** application that ends statistical data from models and receives requests to those models.

Its implementation can be seen in [model_sender_custom.py](model_sender_custom.py) file.

```sh
# To send reply
cd ~/AML-IP-ws/src/AML-IP/amlip_demo_nodes/amlip_collaborative_learning_demo/amlip_collaborative_learning_demo
python3 model_sender_custom.py
```
