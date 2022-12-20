.. include:: ../../../../exports/alias.include
.. include:: ../../../../exports/roles.include

.. _developer_manual_installation_sources_windows_cmake:

#############################################
Windows installation from sources using CMake
#############################################

The instructions for installing the |amlip| using CMake_ from sources and its required
dependencies are provided in this page.
This section explains how to compile |amlip| with CMake_, either
:ref:`locally <windows_local_installation_sl>` or :ref:`globally <windows_global_installation_sl>`.

.. _windows_local_installation_sl:

Local installation
------------------

#.  Open a command prompt, and create a :code:`AML-IP` directory where to download and build |amlip| and
    its dependencies:

    .. code-block:: bash

        mkdir <path\to\user\workspace>\AML-IP
        mkdir <path\to\user\workspace>\AML-IP\src
        mkdir <path\to\user\workspace>\AML-IP\build
        cd <path\to\user\workspace>\AML-IP
        wget https://raw.githubusercontent.com/eProsima/AML-IP/main/amlip.repos
        vcs import src < amlip.repos

#.  Compile all dependencies using CMake_.

    *  `Foonathan memory <https://github.com/foonathan/memory>`_

        .. code-block:: bash

            cd <path\to\user\workspace>\AML-IP
            mkdir build\foonathan_memory_vendor
            cd build\foonathan_memory_vendor
            cmake <path\to\user\workspace>\AML-IP\src\foonathan_memory_vendor -DCMAKE_INSTALL_PREFIX=<path\to\user\workspace>\AML-IP\install ^
                -DBUILD_SHARED_LIBS=ON
            cmake --build . --config Release --target install

    *  `Fast CDR <https://github.com/eProsima/Fast-CDR>`_

        .. code-block:: bash

            cd <path\to\user\workspace>\AML-IP
            mkdir build\fastcdr
            cd build\fastcdr
            cmake <path\to\user\workspace>\AML-IP\src\fastcdr -DCMAKE_INSTALL_PREFIX=<path\to\user\workspace>\AML-IP\install
            cmake --build . --config Release --target install

    *  `Fast DDS <https://github.com/eProsima/Fast-DDS>`_

        .. code-block:: bash

            cd <path\to\user\workspace>\AML-IP
            mkdir build\fastdds
            cd build\fastdds
            cmake <path\to\user\workspace>\AML-IP\src\fastdds -DCMAKE_INSTALL_PREFIX=<path\to\user\workspace>\AML-IP\install ^
                -DCMAKE_PREFIX_PATH=<path\to\user\workspace>\AML-IP\install
            cmake --build . --config Release --target install

    * `Dev Utils <https://github.com/eProsima/dev-utils>`_

        .. code-block:: bash

            # cmake_utils
            cd <path\to\user\workspace>\AML-IP
            mkdir build\cmake_utils
            cd build\cmake_utils
            cmake <path\to\user\workspace>\AML-IP\src\dev-utils\cmake_utils -DCMAKE_INSTALL_PREFIX=<path\to\user\workspace>\AML-IP\install ^
                -DCMAKE_PREFIX_PATH=<path\to\user\workspace>\AML-IP\install
            cmake --build . --config Release --target install

            # cpp_utils
            cd <path\to\user\workspace>\AML-IP
            mkdir build\cpp_utils
            cd build\cpp_utils
            cmake <path\to\user\workspace>\AML-IP\src\dev-utils\cpp_utils -DCMAKE_INSTALL_PREFIX=<path\to\user\workspace>\AML-IP\install ^
                -DCMAKE_PREFIX_PATH=<path\to\user\workspace>\AML-IP\install
            cmake --build . --config Release --target install

    * `DDS Router <https://github.com/eProsima/DDS-Router>`_

        .. code-block:: bash

            # ddsrouter_core
            cd <path\to\user\workspace>\AML-IP
            mkdir build\ddsrouter_core
            cd build\ddsrouter_core
            cmake <path\to\user\workspace>\AML-IP\src\ddsrouter\ddsrouter_core -DCMAKE_INSTALL_PREFIX=<path\to\user\workspace>\AML-IP\install ^
                -DCMAKE_PREFIX_PATH=<path\to\user\workspace>\AML-IP\install
            cmake --build . --config Release --target install

#.  Once all dependencies are installed, install |amlip|:

    .. code-block:: bash

        # amlip_cpp
        cd <path\to\user\workspace>\AML-IP
        mkdir build\amlip_cpp
        cd build\amlip_cpp
        cmake <path\to\user\workspace>\AML-IP\src\amlip\amlip_cpp ^
            -DCMAKE_INSTALL_PREFIX=<path\to\user\workspace>\AML-IP\install -DCMAKE_PREFIX_PATH=<path\to\user\workspace>\AML-IP\install
        cmake --build . --config Release --target install


.. note::

    By default, |amlip| does not compile tests.
    However, they can be activated by downloading and installing `Gtest <https://github.com/google/googletest>`_
    and building with CMake option ``-DBUILD_TESTS=ON``.


.. _windows_global_installation_sl:

Global installation
-------------------

To install |amlip| system-wide instead of locally, remove all the flags that
appear in the configuration steps of :code:`Fast-CDR`, :code:`Fast-DDS`, :code:`AML-IP`, and
:code:`AML-IP`, and change the first in the configuration step of :code:`foonathan_memory_vendor` to the
following:

.. code-block:: bash

    -DCMAKE_INSTALL_PREFIX=/usr/local/ -DBUILD_SHARED_LIBS=ON


.. External links

.. _colcon: https://colcon.readthedocs.io/en/released/
.. _CMake: https://cmake.org
