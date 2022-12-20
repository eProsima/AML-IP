.. include:: ../../../../exports/alias.include
.. include:: ../../../../exports/roles.include

.. _developer_manual_installation_sources_windows_colcon:

##############################################
Windows installation from sources using colcon
##############################################

colcon_ is a command line tool based on CMake_ aimed at building sets of software packages in a tidy and easy way.
The instructions for installing the |amlip| using colcon_ application from sources and its required
dependencies are provided in this page.


Installation
============

This section explains how to install |amlip| using colcon_.

Download eProsima dependencies
------------------------------

#.  Create a :code:`AML-IP` directory and download the :code:`.repos` file that will be used to install
    |amlip| and its dependencies:

    .. code-block:: bash

        mkdir <path\to\user\workspace>\AML-IP
        cd <path\to\user\workspace>\AML-IP
        mkdir src
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

        colcon build --packages-up-to amlip_cpp

.. note::

    Being based on CMake_, it is possible to pass the CMake_ configuration options to the :code:`colcon build`
    command. For more information on the specific syntax, please refer to the
    `CMake specific arguments <https://colcon.readthedocs.io/en/released/reference/verb/build.html#cmake-specific-arguments>`_
    page of the colcon_ manual.
    For more details about the available CMake_ options, please refer to the :ref:`cmake_options` section.


Source installation
===================

To source the installation of the |amlip| previously built in order to link it to another built or to use
its tools, use the following command:

* Command prompt:

.. code-block:: bat

    install/setup.bat

* PowerShell:

.. code-block:: bash

    install/setup.ps1


.. External links

.. _colcon: https://colcon.readthedocs.io/en/released/
.. _CMake: https://cmake.org
