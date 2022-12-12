.. include:: ../../../../exports/alias.include
.. include:: ../../../../exports/roles.include

.. _developer_manual_installation_sources_linux_cmake:

###########################################
Linux installation from sources using CMake
###########################################

The instructions for installing the |amlip| using CMake_ from sources and its required
dependencies are provided in this page.
This section explains how to compile |amlip| with CMake_, either
:ref:`locally <local_installation_sl>` or :ref:`globally <global_installation_sl>`.

.. _local_installation_sl:

Local installation
------------------

#.  Create a :code:`AML-IP` directory where to download and build |amlip| and its dependencies:

    .. code-block:: bash

        mkdir -p ~/AML-IP/src
        mkdir -p ~/AML-IP/build
        cd ~/AML-IP
        wget https://raw.githubusercontent.com/eProsima/AML-IP/main/amlip.repos
        vcs import src < amlip.repos

#.  Compile all dependencies using CMake_.

    * `Foonathan memory <https://github.com/foonathan/memory>`_

        .. code-block:: bash

            cd ~/AML-IP
            mkdir build/foonathan_memory_vendor
            cd build/foonathan_memory_vendor
            cmake ~/AML-IP/src/foonathan_memory_vendor -DCMAKE_INSTALL_PREFIX=~/AML-IP/install -DBUILD_SHARED_LIBS=ON
            cmake --build . --target install

    * `Fast CDR <https://github.com/eProsima/Fast-CDR>`_

        .. code-block:: bash

            cd ~/AML-IP
            mkdir build/fastcdr
            cd build/fastcdr
            cmake ~/AML-IP/src/fastcdr -DCMAKE_INSTALL_PREFIX=~/AML-IP/install
            cmake --build . --target install

    * `Fast DDS <https://github.com/eProsima/Fast-DDS>`_

        .. code-block:: bash

            cd ~/AML-IP
            mkdir build/fastdds
            cd build/fastdds
            cmake ~/AML-IP/src/fastdds -DCMAKE_INSTALL_PREFIX=~/AML-IP/install -DCMAKE_PREFIX_PATH=~/AML-IP/install
            cmake --build . --target install

    * `Dev Utils <https://github.com/eProsima/dev-utils>`_

        .. code-block:: bash

            # cmake_utils
            cd ~/AML-IP
            mkdir build/cmake_utils
            cd build/cmake_utils
            cmake ~/AML-IP/src/dev-utils/cmake_utils -DCMAKE_INSTALL_PREFIX=~/AML-IP/install -DCMAKE_PREFIX_PATH=~/AML-IP/install
            cmake --build . --target install

            # cpp_utils
            cd ~/AML-IP
            mkdir build/cpp_utils
            cd build/cpp_utils
            cmake ~/AML-IP/src/dev-utils/cpp_utils -DCMAKE_INSTALL_PREFIX=~/AML-IP/install -DCMAKE_PREFIX_PATH=~/AML-IP/install
            cmake --build . --target install

    * `DDS Router <https://github.com/eProsima/DDS-Router>`_

        .. code-block:: bash

            # ddsrouter_core
            cd ~/AML-IP
            mkdir build/ddsrouter_core
            cd build/ddsrouter_core
            cmake ~/AML-IP/src/ddsrouter/ddsrouter_core -DCMAKE_INSTALL_PREFIX=~/AML-IP/install -DCMAKE_PREFIX_PATH=~/AML-IP/install
            cmake --build . --target install


#.  Once all dependencies are installed, install |amlip|:

    .. code-block:: bash

        # amlip_cpp
        cd ~/AML-IP
        mkdir build/amlip_cpp
        cd build/amlip_cpp
        cmake ~/AML-IP/src/amlip/amlip_cpp -DCMAKE_INSTALL_PREFIX=~/AML-IP/install -DCMAKE_PREFIX_PATH=~/AML-IP/install
        cmake --build . --target install

        # amlip_swig
        cd ~/AML-IP
        mkdir build/amlip_swig
        cd build/amlip_swig
        cmake ~/AML-IP/src/amlip/amlip_swig -DCMAKE_INSTALL_PREFIX=~/AML-IP/install -DCMAKE_PREFIX_PATH=~/AML-IP/install
        cmake --build . --target install

        # amlip_py
        cd ~/AML-IP
        mkdir build/amlip_py
        cd build/amlip_py
        cmake ~/AML-IP/src/amlip/tools/amlip_py -DCMAKE_INSTALL_PREFIX=~/AML-IP/install -DCMAKE_PREFIX_PATH=~/AML-IP/install
        cmake --build . --target install

.. note::

    By default, |amlip| does not compile tests.
    However, they can be activated by downloading and installing `Gtest <https://github.com/google/googletest>`_
    and building with CMake option ``-DBUILD_TESTS=ON``.


.. _global_installation_sl:

Global installation
-------------------

To install |amlip| system-wide instead of locally, remove all the flags that
appear in the configuration steps of :code:`Fast-CDR`, :code:`Fast-DDS`,  :code:`DDS-Router`, and
:code:`AML-IP`, and change the first in the configuration step of :code:`foonathan_memory_vendor` to the
following:

.. code-block:: bash

    -DCMAKE_INSTALL_PREFIX=/usr/local/ -DBUILD_SHARED_LIBS=ON


.. External links

.. _colcon: https://colcon.readthedocs.io/en/released/
.. _CMake: https://cmake.org
