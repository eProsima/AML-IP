# eProsima AML-IP TensorFlow Inference Demo

This is a package containing demos of AML-IP node execution.
In this package there are severat executables:

- [Async Edge](amlip_tensorflow_inference_demo/edge_node_async.py) Async Edge Node execution
- [Async Inference](amlip_tensorflow_inference_demo/inference_node_async.py) Async Inference Node execution
- [Sync Edge](amlip_tensorflow_inference_demo/edge_node_sync.py) Sync Edge Node execution
- [Sync Inference](amlip_tensorflow_inference_demo/inference_node_sync.py) Sync Inference Node execution

---

## Index

- [eProsima AML-IP TensorFlow Inference Demo](#eprosima-aml-ip-tensorflow-inference-demo)
  - [Index](#index)
  - [Prepare Workspace](#prepare-workspace)
  - [Prerequisites](#prerequisites)
  - [Build Project](#build-project)
  - [Run demo](#run-demo)
    - [Async Edge Node](#async-edge-node)
    - [Async Inference Node](#async-inference-node)
  - [Docker](#docker)
  - [Synchronous Nodes simulated](#synchronous-nodes-simulated)
    - [Sync Main Node](#sync-edge-node)
    - [Sync Computing Node](#sync-inference-node)

<img src="../../.figures/demos/tensor_inference_demo.png"/>

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

## Prerequisites

The demo requires the following tools to be installed in the system:

```bash
sudo apt install -y  swig alsa-utils libopencv-dev
pip3 install -U pyttsx3 opencv-python
curl https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -o Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh
# For changes to take effect, close and re-open your current shell.
conda create --name tf python=3.9
conda install -c conda-forge cudatoolkit=11.8.0
mkdir -p $CONDA_PREFIX/etc/conda/activate.d
echo 'CUDNN_PATH=$(dirname $(python3 -c "import nvidia.cudnn;print(nvidia.cudnn.__file__)"))' >> $CONDA_PREFIX/etc/conda/activate.d/env_vars.sh
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CONDA_PREFIX/lib/:$CUDNN_PATH/lib' >> $CONDA_PREFIX/etc/conda/activate.d/env_vars.sh
source $CONDA_PREFIX/etc/conda/activate.d/env_vars.sh
```

Ensure that you have TensorFlow and TensorFlow Hub installed in your Python environment before proceeding.
You can install them using pip by executing the following commands:

```bash
pip3 install tensorflow tensorflow-hub tensorflow-object-detection-api nvidia-cudnn-cu11==8.6.0.163 protobuf==3.20.*
```

Additionally, it is required to obtain the TensorFlow model from `TensorFlow Hub <https://tfhub.dev/>`_, follow the steps below:

```bash
cd ~/src/AML-IP/amlip_demo_modes/amlip_tensorflow_inference_demo/resource/tensorflow/models/
wget -O centernet_hourglass_512x512_kpts_1.tar.gz https://tfhub.dev/tensorflow/centernet/hourglass_512x512_kpts/1?tf-hub-format=compressed
mkdir centernet_hourglass_512x512_kpts_1
tar -xvf centernet_hourglass_512x512_kpts_1.tar.gz -C ./centernet_hourglass_512x512_kpts_1
```

---

## Build Project

In order to build the project and be able to operate with this demo, follow these instructions:

```sh
colcon build --packages-up-to amlip_demo_nodes
source install/setup.bash
```

---

## Run demo

Run Edge and Inference nodes in two terminals.
In each one, source the workspace and launch a node.

### Edge Node

```bash
source ~/AML-IP-ws/install/setup.bash
cd ~/AML-IP-ws/src/AML-IP/amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo
python3 edge_node_async.py
```

Expected output:

```bash
Edge Node AMLEdgeNode.fb.d4.38.13 ready.
Edge Node AMLEdgeNode.fb.d4.38.13 sending data.

Edge Node received inference from AMLInferenceNode.b8.34.4d.a3
Edge Node received inference:
Box [(0.15590962767601013, 0.21641747653484344), (0.7388607263565063, 0.7326743006706238)] bicycle: 97%
Box [(0.16968876123428345, 0.38129815459251404), (0.403958797454834, 0.9422630071640015)] dog: 92%
Box [(0.6158109307289124, 0.13117200136184692), (0.9053990244865417, 0.2978983521461487)] truck: 53%
Box [(0.6158109307289124, 0.13117200136184692), (0.9053990244865417, 0.2978983521461487)] car: 48%
Box [(0.8892407417297363, 0.19558095932006836), (0.933372974395752, 0.2684069573879242)] potted plant: 34%
Box [(0.0753115713596344, 0.15651819109916687), (0.13415342569351196, 0.22736744582653046)] motorcycle: 32%

Edge Node AMLEdgeNode.fb.d4.38.13 closing.
```

### Inference Node

```bash
source ~/AML-IP-ws/install/setup.bash
cd ~/AML-IP-ws/src/AML-IP/amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo
python3 inference_node_async.py
```

Expected output:

```bash
2023-02-14 14:50:42.711797: I tensorflow/core/platform/cpu_feature_guard.cc:193] This TensorFlow binary is optimized with oneAPI Deep Neural Network Library (oneDNN) to use the following CPU instructions in performance-critical operations:  AVX2 FMA
To enable them in other operations, rebuild TensorFlow with the appropriate compiler flags.
Inference Node AMLInferenceNode.b8.34.4d.a3 ready.
Model Handle at TensorFlow Hub: /home/user/AML-IP/src/amlipdemos/amlip_demos/resource/tensorflow/models/centernet_hourglass_512x512_kpts_1
loading model...
WARNING:absl:Importing a function (__inference_batchnorm_layer_call_and_return_conditional_losses_42408) with ops with unsaved custom gradients. Will likely fail if a gradient is requested.
WARNING:absl:Importing a function (__inference_batchnorm_layer_call_and_return_conditional_losses_209416) with ops with unsaved custom gradients. Will likely fail if a gradient is requested.
WARNING:absl:Importing a function (__inference_batchnorm_layer_call_and_return_conditional_losses_220336) with ops with unsaved custom gradients. Will likely fail if a gradient is requested.
...
WARNING:absl:Importing a function (__inference_batchnorm_layer_call_and_return_conditional_losses_55827) with ops with unsaved custom gradients. Will likely fail if a gradient is requested.
WARNING:absl:Importing a function (__inference_batchnorm_layer_call_and_return_conditional_losses_56488) with ops with unsaved custom gradients. Will likely fail if a gradient is requested.
model loaded!
Selected model:tensorflow
2023-02-14 14:51:14.165305: W tensorflow/core/grappler/optimizers/loop_optimizer.cc:907] Skipping loop optimization for Merge node with control input: StatefulPartitionedCall/cond/then/_918/cond/Assert_2/AssertGuard/branch_executed/_1123
Inference ready!
Sending inference:
Box [(0.15590962767601013, 0.21641747653484344), (0.7388607263565063, 0.7326743006706238)] bicycle: 97%
Box [(0.16968876123428345, 0.38129815459251404), (0.403958797454834, 0.9422630071640015)] dog: 92%
Box [(0.6158109307289124, 0.13117200136184692), (0.9053990244865417, 0.2978983521461487)] truck: 53%
Box [(0.6158109307289124, 0.13117200136184692), (0.9053990244865417, 0.2978983521461487)] car: 48%
Box [(0.8892407417297363, 0.19558095932006836), (0.933372974395752, 0.2684069573879242)] potted plant: 34%
Box [(0.0753115713596344, 0.15651819109916687), (0.13415342569351196, 0.22736744582653046)] motorcycle: 32%

Inference sent to client AMLEdgeNode.fb.d4.38.13.
```

---

## Synchronous Nodes simulated

There are some demos that contains old examples on Synchronous Nodes.

### Sync Edge Node

Its implementation can be seen in [edge_node_sync.py](amlip_tensorflow_inference_demo/edge_node_sync.py) file.

```sh
cd ~/AML-IP-ws/src/AML-IP/amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo
python3 edge_node_sync.py
```

### Sync Inference Node

Its implementation can be seen in [inference_node_sync.py](amlip_tensorflow_inference_demo/inference_node_sync.PY) file.

```sh
cd ~/AML-IP-ws/src/AML-IP/amlip_demo_nodes/amlip_tensorflow_inference_demo/amlip_tensorflow_inference_demo
python3 inference_node_sync.py
```
