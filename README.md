# eProsima AML-IP

**Algebraic Machine Learning - Integrating Platform** project.

<a href="http://www.eprosima.com"><img src="https://encrypted-tbn3.gstatic.com/images?q=tbn:ANd9GcSd0PDlVz1U_7MgdTe0FRIWD0Jc9_YH-gGi0ZpLkr-qgCI6ZEoJZ5GBqQ" align="left" hspace="8" vspace="2" width="100" height="100" ></a>

[![License](https://img.shields.io/github/license/eProsima/AML-IP.svg)](https://opensource.org/licenses/Apache-2.0)
[![Releases](https://img.shields.io/github/v/release/eProsima/AML-IP?sort=semver)](https://github.com/eProsima/AML-IP/releases)
[![Issues](https://img.shields.io/github/issues/eProsima/AML-IP.svg)](https://github.com/eProsima/AML-IP/issues)
[![Forks](https://img.shields.io/github/forks/eProsima/AML-IP.svg)](https://github.com/eProsima/AML-IP/network/members)
[![Stars](https://img.shields.io/github/stars/eProsima/AML-IP.svg)](https://github.com/eProsima/AML-IP/stargazers)
[![test](https://github.com/eProsima/AML-IP/actions/workflows/test.yml/badge.svg)](https://github.com/eProsima/AML-IP/actions/workflows/test.yml)
[![codecov](https://codecov.io/gh/eProsima/AML-IP/branch/main/graph/badge.svg?token=M5Y82MGCO7)](https://codecov.io/gh/eProsima/AML-IP)
[![Documentation Status](https://readthedocs.org/projects/aml-ip/badge/?version=latest)](https://aml-ip.readthedocs.io/en/latest/)

*eProsima AML-IP* is a communications framework in charge of data exchange between Algebraic Machine Learning (AML)
nodes through local or remote networks.
It is designed to allow non-experts users to create and manage a cluster of AML nodes
to exploit the distributed and concurrent learning capabilities of AML.
Thus, AML-IP is a communication framework that makes the transport protocols abstracted from the user,
creating a platform that communicates each node without requiring the user to be concerned about communication issues.
It also allows the creation of complex distributed networks with one or multiple users working on the same problem.

---

## Documentation

You can access the documentation online, which is hosted on [Read the Docs](https://aml-ip.readthedocs.io).

* [Introduction](https://aml-ip.readthedocs.io/en/latest/rst/formalia/titlepage.html)
* [Getting Started](https://aml-ip.readthedocs.io/en/latest/rst/getting_started/project_overview.html)
* [User Manual](https://aml-ip.readthedocs.io/en/latest/rst/user_manual/scenarios/scenarios.html)
* [Demos](https://aml-ip.readthedocs.io/en/latest/rst/demo/workload_distribution.html)
* [Developer Manual](https://aml-ip.readthedocs.io/en/latest/rst/developer_manual/installation/sources/linux/linux.html)
* [Release Notes](https://aml-ip.readthedocs.io/en/latest/rst/notes/notes.html)

## Installation Guide

The instructions for installing *AML-IP* application from sources and its required dependencies are provided in the [documentation installation guide](https://aml-ip.readthedocs.io/en/latest/rst/installation/linux.html) for `Linux` ow `Windows`.

---

## Project status

**This project is a work in progress and the following features presented here will be extended, updated, and improved in future versions.**

### AML-IP Nodes

So far, these are the Nodes supported:

* `StatusNode` [C++, python] [documentation](https://aml-ip.readthedocs.io/en/latest/rst/user_manual/nodes/status.html)
* `MainNode` [C++, python] [documentation](https://aml-ip.readthedocs.io/en/latest/rst/user_manual/nodes/main.html)
* `ComputingNode` [C++, python] [documentation](https://aml-ip.readthedocs.io/en/latest/rst/user_manual/nodes/computing.html)

### AML-IP Scenarios

So far, these are the Nodes supported:

* `Monitor Network` [documentation](https://aml-ip.readthedocs.io/en/latest/rst/user_manual/scenarios/monitor_state.html)
* `Workload Distribution` [documentation](https://aml-ip.readthedocs.io/en/latest/rst/user_manual/scenarios/workload_distribution.html)

### Run a demo

Check the following [documentation](https://aml-ip.readthedocs.io/en/latest/rst/demo/workload_distribution.html) for a complete tutorial on how to execute a demo.

---

## Architecture

This repository is divided in sub-packages with different targets:

* `amlip_cpp` Main definition and implementation of the project logic library. C++ API provided.
* `amlip_swig` Binding from `amlip_cpp` to a Python API.
* `amlip_py` Python API.
* `amlip_docs` Documentation with sphinx.
* `amlip_demo` Demonstrators and examples of *AML-IP*.

### Dependencies

*AML-IP* depends on several standard or eProsima projects:

* Third-party libraries
  * Dependencies to download, compile and install project:
    * `wget`
    * `git`
  * Dependencies to install project:
    * `CMake`
    * `gcc`
    * `python3`
    * `pip`
    * `colcon` [optional]
  * Library dependencies:
    * `Asio`
    * `TinyXML2`
    * `yaml-cpp`
    * `OpenSSL` [ony with security]
    * `GTest` [ony for test]
* eProsima libraries
  * `fastcdr` Fast CDR for message serialization and deserialization.
  * `fastrtps` Fast DDS library for DDS communication.
  * `cmake_utils` CMake utilities library.
  * `cpp_utils` C++ utilities library.
  * `ddsrouter_core` DDS-Router core library.
