# eProsima AML-IP

**Algebraic Machine Learning - Integrating Platform** project.

> :warning: **TODO** badges

*eProsima AML-IP* is an end-user software application with a Python API that allow to instantiate AML-IP nodes
and interact with them.
These nodes are intercommunicated by *DDS* protocol powered by *Fast-DDS* and allow the user to communicate
its AML data.

## AML-IP Nodes

> :warning: **TODO**

## Documentation

> :warning: **TODO**

## Installation Guide

> :warning: **TODO**

### Requirements

*eProsima AML-IP* requires the following tools to be installed in the system:

* [CMake](https://cmake.org/),
  [g++](https://gcc.gnu.org/),
  [python3](https://www.python.org/),
  [pip](https://pypi.org/project/pip/),
  [wget](https://www.gnu.org/software/wget/),
  [git](https://git-scm.com/)
* [Colcon](https://colcon.readthedocs.io/en/released/) [optional, not required for CMake-only installation]
* [Gtest](https://github.com/google/googletest) [for test only]

#### CMake, g++, pip, wget and git

These packages provide the tools required to install AML-IP and its dependencies from command line. Install
[CMake](https://cmake.org/), [g++](https://gcc.gnu.org/), [pip](https://pypi.org/project/pip/), [wget](https://www.gnu.org/software/wget/) and [git](https://git-scm.com/) using the package manager of the appropriate Linux distribution. For
example, on Ubuntu use the command:

```bash
sudo apt install cmake g++ pip wget git
```

#### Colcon

[colcon](https://colcon.readthedocs.io/en/released/) is a command line tool based on [CMake](https://cmake.org/) aimed at building sets of software packages. Install the ROS 2 development tools ([colcon](https://colcon.readthedocs.io/en/released/) and [vcstool](https://pypi.org/project/vcstool/)) by executing the following command:

```bash
pip3 install -U colcon-common-extensions vcstool
```

If this fails due to an Environment Error, add the `--user` flag to the `pip3` installation command.

#### Gtest

[Gtest](https://github.com/google/googletest) is a unit testing library for C++. By default, *AML-IP* does not
compile tests. It is possible to activate them with the opportune [CMake options](https://colcon.readthedocs.io/en/released/reference/verb/build.html#cmake-options) when calling [colcon](https://colcon.readthedocs.io/en/released/) or
[CMake](https://cmake.org/). For a detailed description of the Gtest installation process, please refer to the
[Gtest Installation Guide](https://github.com/google/googletest).

### Dependencies

#### Asio and TinyXML2 libraries

Asio is a cross-platform C++ library for network and low-level I/O programming, which provides a consistent asynchronous
model. TinyXML2 is a simple, small and efficient C++ XML parser. Install these libraries using the package manager of
the appropriate Linux distribution. For example, on Ubuntu use the command:

```bash
sudo apt install libasio-dev libtinyxml2-dev
```

#### OpenSSL

[OpenSSL](https://www.openssl.org/) is a robust toolkit for the TLS and SSL protocols and a general-purpose cryptography
library. Install OpenSSL using the package manager of the appropriate Linux distribution. For example, on Ubuntu use the
command:

```bash
sudo apt install libssl-dev
```

#### SWIG

`amlip_bindings` requires `SWIG` software.
[SWIG](https://www.swig.org/) is a software development tool that connects programs written in C and C++
with a variety of high-level programming languages.
Install yaml-cpp using the package manager of the appropriate Linux distribution. For example, on
Ubuntu use the command:

```bash
sudo apt update
sudo apt install -y \
    swig \
```

#### eProsima dependencies

If it already exists in the system an installation of *Fast DDS* library with version greater than *2.4.0*, just source
this library when building the *AML-IP* application by using the command:

```bash
source <fastdds-installation-path>/install/setup.bash
```

In other case, just download *Fast DDS* project from sources and build it together with *AML-IP* using colcon as it
is explained in the following section.

### Colcon installation

1. Create a `AML-IP` directory and download the `.repos` file that will be used to install *AML-IP* and its dependencies:

```bash
mkdir -p ~/AML-IP/src
cd ~/AML-IP
wget https://raw.githubusercontent.com/eProsima/AML-IP/main/ddsrouter.repos
vcs import src < ddsrouter.repos
```

2. Build the packages:

```bash
colcon build
```

This repository holds several colcon packages.
These packages are:

* `amlip_cpp`       : C++ library with main AML-IP functionality to create Nodes and interact with them.
* `amlip_bindings`  : Python API autogenerated by SWIG from `amlip_cpp` API.
* `amlip_py`        : Python library with main AML-IP functionality to create Nodes and interact with them.
* `amlip_docs`      : Documentation package built with sphinx

> *NOTE:* Those packages could be installed and use independently (according with each package dependency).
  In order to compile only a package and its dependencies, use the colcon argument `--packages-up-to <package>`.
  In order to explicitly skip some of these packages, use the colcon argument
  `--packages-skip <package1> [<package2> ...]`.

### Run an application

> :warning: **TODO**

### Testing

By default, *AML-IP* does not compile tests. However, they can be activated by downloading and installing
[Gtest](https://github.com/google/googletest) and building this project with CMake option `-DBUILD_TESTS=ON`.
Once done, tests can be run with the following command:

```bash
colcon test --packages-select-regex amlip --event-handler=console_direct+
```
