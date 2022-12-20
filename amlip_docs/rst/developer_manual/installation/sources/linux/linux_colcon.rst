.. include:: ../../../../exports/alias.include
.. include:: ../../../../exports/roles.include

.. _developer_manual_installation_sources_linux_colcon:

############################################
Linux installation from sources using colcon
############################################

colcon_ is a command line tool based on CMake_ aimed at building sets of software packages in a tidy and easy way.
The instructions for installing the |amlip| using colcon_ application from sources and its required
dependencies are provided in this page.


Installation
============

Follow the instructions below to build |eamlip|, after making sure all required dependencies are installed in your system (:ref:`developer_manual_installation_sources_linux_dependencies`).

Download eProsima dependencies
------------------------------

#.  Create a :code:`AML-IP` directory and download the :code:`.repos` file that will be used to install
    |amlip| and its dependencies:

    .. code-block:: bash

        mkdir -p ~/AML-IP/src
        cd ~/AML-IP
        wget https://raw.githubusercontent.com/eProsima/AML-IP/main/amlip.repos
        vcs import src < amlip.repos

    .. note::

        In case there are already some eProsima libraries installed in the system,
        it is not required to download and build every dependency in the :code:`.repos` file, but just those projects that are not already in the system.
        Refer to section :ref:`eprosima_dependencies` in order to check how to source those libraries.

Build packages
--------------

#.  Build the packages:

    .. code-block:: bash

        colcon build

.. note::

    Not all the sub-packages of all the dependencies are required.
    In order to build only the packages required, use the colcon_ option :code:`--packages-up-to <package-to-build>`.
    *e.g. the* |amlip| *C++ library is completely built using* :code:`--packages-up-to amlip_cpp`
    For more details about the colcon_ available arguments, please refer to `packages selection <https://colcon.readthedocs.io/en/released/reference/package-selection-arguments.html>`_
    page of the colcon_ manual.

.. note::

    Being based on CMake_, it is possible to pass the CMake_ configuration options to the :code:`colcon build`
    command. For more information on the specific syntax, please refer to the
    `CMake specific arguments <https://colcon.readthedocs.io/en/released/reference/verb/build.html#cmake-specific-arguments>`_
    page of the colcon_ manual.
    For more details about the available CMake_ options, please refer to the :ref:`cmake_options` section.


Run Tests
---------

Tests are not automatically built within the |amlip| project.
Use CMake_ option `BUILD_TESTS` when building the project in order to activate tests.
This could also be done by a `colcon.meta file <https://colcon.readthedocs.io/en/released/user/configuration.html#meta-files>`_
to only activate tests in the desired packages.

#.  Build the packages with tests:

    .. code-block:: bash

        colcon build --packages-select-regex amlip --cmake-args "-DBUILD_TESTS=ON"


#.  Run tests. Use :code:`--packages-select <package-name>` to only execute tests of a specific package:

    .. code-block:: bash

        colcon test --event-handlers=console_direct+ --packages-select amlip_cpp


Source installation
===================

To source the installation of the previously built |amlip| (in order to use its tools or link against it), use the following command:

.. code-block:: bash

    source install/setup.bash


.. External links

.. _colcon: https://colcon.readthedocs.io/en/released/
.. _CMake: https://cmake.org
