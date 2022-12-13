.. include:: ../../../../exports/alias.include
.. include:: ../../../../exports/roles.include

.. _developer_manual_installation_sources_linux:

###############################
Linux installation from sources
###############################

Sub-packages
============

The |amlip| is constituted of several sub-packages.
Depending on the use of those packages, some or all of them must be built.
These are the packages of |amlip| and their dependency between each other:

.. list-table::
    :header-rows: 1

    *   - Sub-package
        - Description
        - Depends on

    *   - |amlip_cpp|
        - Main C++ library with the implementation and API to create |amlip| Nodes.
        -

    *   - |amlip_swig|
        - Project to auto-generate a Python library from |amlip_cpp|.
        - |amlip_cpp|

    *   - |amlip_py|
        - Main Python library with API to create AML-IP Nodes.
        - |amlip_swig|

    *   - |amlip_docs|
        - Sphinx documentation project.
        -

Dependencies
============

These are the dependencies required in the system before building |amlip| from sources.

* :ref:`cmake_gcc_pip_wget_git_sl_installation`
* :ref:`asiotinyxml2_installation`
* :ref:`openssl_installation`
* :ref:`colcon_installation` [optional]
* :ref:`gtest_installation` [for test only]
* :ref:`eprosima_dependencies`


.. _cmake_gcc_pip_wget_git_sl_installation:

CMake, g++, pip, wget and git
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

These packages provide the tools required to install |amlip| and its dependencies from command line.
Install CMake_, `g++ <https://gcc.gnu.org/>`_, pip_, wget_ and git_ using the package manager of the appropriate
Linux distribution. For example, on Ubuntu use the command:

.. code-block:: bash

    sudo apt install cmake g++ pip wget git


.. _asiotinyxml2_installation:

Asio and TinyXML2 libraries
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Asio is a cross-platform C++ library for network and low-level I/O programming, which provides a consistent
asynchronous model.
TinyXML2 is a simple, small and efficient C++ XML parser.
Install these libraries using the package manager of the appropriate Linux distribution.
For example, on Ubuntu use the command:

.. code-block:: bash

    sudo apt install libasio-dev libtinyxml2-dev


.. _openssl_installation:

OpenSSL
^^^^^^^

OpenSSL is a robust toolkit for the TLS and SSL protocols and a general-purpose cryptography library.
Install OpenSSL_ using the package manager of the appropriate Linux distribution.
For example, on Ubuntu use the command:

.. code-block:: bash

   sudo apt i_installationnstall libssl-dev


.. _colcon_installation:

Colcon
^^^^^^

Install the ROS 2 development tools (colcon_ and vcstool_) by executing the following command:

.. code-block:: bash

    pip3 install -U colcon-common-extensions vcstool

.. note::

    If this fails due to an Environment Error, add the :code:`--user` flag to the pip_ installation command.


.. _gtest_installation:

Gtest
^^^^^

Gtest_ is a unit testing library for C++.
For a detailed description of the Gtest_ installation process, please refer to the
`Gtest Installation Guide <https://github.com/google/googletest>`_.

If using colcon_, it is also possible to clone the Gtest_ Github repository into the |ddsrouter|
workspace and compile it as a dependency package.
Add this new package to :code:`.repos` file before importing the sources:

.. code-block:: bash

    googletest-distribution:
        type: git
        url: https://github.com/google/googletest.git
        version: release-1.12.1


.. _eprosima_dependencies:

eProsima dependencies
^^^^^^^^^^^^^^^^^^^^^

These are the eProsima libraries required for building |amlip|:

- ``foonathan_memory_vendor``, an STL compatible C++ memory allocation library.
- ``fastcdr``, a C++ library that serializes according to the standard CDR serialization mechanism.
- ``fastrtps``, the core library of eProsima Fast DDS library.
- ``cmake_utils``, an eProsima utilities library for CMake.
- ``cpp_utils``, an eProsima utilities library for C++.
- ``ddsrouter_core``, the eProsima DDS Router library C++.

If it already exists in the system an installation of these libraries there is no need to build them again,
just source them when building the |amlip|.
If using CMake_, add the already libraries installation paths to :code:`LD_LIBRARY_PATH`.
If using colcon_, use the following command to source them:

.. code-block:: bash

    source <eprosima-dependencies-installation-path>/install/setup.bash


Installation methods
====================

There are two main possibilities to build |amlip| from sources in Linux.
One of them uses CMake_ and the other colcon_, an auto-build framework.
**colcon_ version is advice for non advance users as it is easier and neater.**

.. toctree::
    :maxdepth: 1

    colcon <linux_colcon>
    CMake <linux_cmake>

.. External links

.. _colcon: https://colcon.readthedocs.io/en/released/
.. _CMake: https://cmake.org
.. _pip: https://pypi.org/project/pip/
.. _wget: https://www.gnu.org/software/wget/
.. _git: https://git-scm.com/
.. _OpenSSL: https://www.openssl.org/
.. _Gtest: https://github.com/google/googletest
.. _vcstool: https://pypi.org/project/vcstool/
