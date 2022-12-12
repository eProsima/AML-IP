.. include:: ../../../../exports/alias.include
.. include:: ../../../../exports/roles.include

.. _developer_manual_installation_sources_windows:

#################################
Windows installation from sources
#################################

AML-IP Sub-packages
===================

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

The installation of *eProsima Fast DDS* in a Windows environment from sources requires the following tools to be
installed in the system:

* :ref:`windows_sources_visual_studio`
* :ref:`windows_sources_chocolatey`
* :ref:`windows_sources_cmake_pip3_wget_git`
* :ref:`windows_sources_asiotinyxml2`
* :ref:`windows_sources_openssl`
* :ref:`windows_sources_colcon_install` [optional]
* :ref:`windows_sources_gtest` [for test only]
* :ref:`windows_sources_eprosima_dependencies`

.. _windows_sources_visual_studio:

Visual Studio
^^^^^^^^^^^^^

`Visual Studio <https://visualstudio.microsoft.com/>`_ is required to
have a C++ compiler in the system. For this purpose, make sure to check the
:code:`Desktop development with C++` option during the Visual Studio installation process.

If Visual Studio is already installed but the Visual C++ Redistributable packages are not,
open Visual Studio and go to :code:`Tools` -> :code:`Get Tools and Features` and in the :code:`Workloads` tab enable
:code:`Desktop development with C++`. Finally, click :code:`Modify` at the bottom right.


.. _windows_sources_chocolatey:

Chocolatey
^^^^^^^^^^

Chocolatey is a Windows package manager. It is needed to install some of *eProsima Fast DDS*'s dependencies.
Download and install it directly from the `website <https://chocolatey.org/>`_.


.. _windows_sources_cmake_pip3_wget_git:

CMake, pip3, wget and git
^^^^^^^^^^^^^^^^^^^^^^^^^

These packages provide the tools required to install *eProsima Fast DDS* and its dependencies from command line.
Download and install CMake_, pip3_, wget_ and git_ by following the instructions detailed in the respective
websites.
Once installed, add the path to the executables to the :code:`PATH` from the
*Edit the system environment variables* control panel.


.. _windows_sources_asiotinyxml2:

Asio and TinyXML2 libraries
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Asio is a cross-platform C++ library for network and low-level I/O programming, which provides a consistent
asynchronous model.
TinyXML2 is a simple, small and efficient C++ XML parser.
They can be downloaded directly from the links below:

* `Asio <https://github.com/ros2/choco-packages/releases/download/2020-02-24/asio.1.12.1.nupkg>`_
* `TinyXML2 <https://github.com/ros2/choco-packages/releases/download/2020-02-24/tinyxml2.6.0.0.nupkg>`_

After downloading these packages, open an administrative shell with *PowerShell* and execute the following command:

.. code-block:: bash

    choco install -y -s <PATH_TO_DOWNLOADS> asio tinyxml2

where :code:`<PATH_TO_DOWNLOADS>` is the folder into which the packages have been downloaded.


.. _windows_sources_openssl:

OpenSSL
^^^^^^^

OpenSSL_ is a robust toolkit for the TLS and SSL protocols and a general-purpose cryptography library.
Download and install the latest OpenSSL version for Windows at this
`link <https://slproweb.com/products/Win32OpenSSL.html>`_.
After installing, add the environment variable :code:`OPENSSL_ROOT_DIR` pointing to the installation root directory.

For example:

.. code-block:: bash

   OPENSSL_ROOT_DIR=C:\Program Files\OpenSSL-Win64


.. _windows_sources_colcon_install:

Colcon
^^^^^^

colcon_ is a command line tool based on CMake_ aimed at building sets of software packages.
Install the ROS 2 development tools (colcon_ and vcstool_) by executing the following command:

.. code-block:: bash

    pip3 install -U colcon-common-extensions vcstool

.. note::

    If this fails due to an Environment Error, add the :code:`--user` flag to the :code:`pip3` installation command.


.. _windows_sources_gtest:

Gtest
^^^^^

Gtest is a unit testing library for C++.
By default, |ddsrouter| does not compile tests.
It is possible to activate them with the opportune
`CMake options <https://colcon.readthedocs.io/en/released/reference/verb/build.html#cmake-options>`_
when calling colcon_ or CMake_.
For more details, please refer to the :ref:`cmake_options` section.

Run the following commands on your workspace to install Gtest.

.. code-block:: bash

    git clone https://github.com/google/googletest.git
    cmake -DCMAKE_INSTALL_PREFIX='C:\Program Files\gtest' -Dgtest_force_shared_crt=ON -DBUILD_GMOCK=ON ^
        -B build\gtest -A x64 -T host=x64 googletest
    cmake --build build\gtest --config Release --target install

or refer to the
`Gtest Installation Guide <https://github.com/google/googletest>`_ for a detailed description of the Gtest installation
process.


.. _windows_sources_eprosima_dependencies:

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
If using CMake_, add the already libraries installation paths to :code:`PATH`.
If using colcon_, use the following command to source them:

.. code-block:: bash

    source <eprosima-dependencies-installation-path>/install/setup.bash


Installation methods
====================

There are two main possibilities to build |amlip| from sources in Windows.
One of them uses CMake_ and the other colcon_, an auto-build framework.
**colcon_ version is advice for non advance users as it is easier and neater.**

.. toctree::
    :maxdepth: 1

    colcon <windows_colcon>
    CMake <windows_cmake>

.. External links

.. _colcon: https://colcon.readthedocs.io/en/released/
.. _CMake: https://cmake.org
.. _pip3: https://docs.python.org/3/installing/index.html
.. _wget: https://www.gnu.org/software/wget/
.. _git: https://git-scm.com/
.. _OpenSSL: https://www.openssl.org/
.. _Gtest: https://github.com/google/googletest
.. _vcstool: https://pypi.org/project/vcstool/
