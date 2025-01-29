.. include:: ../exports/alias.include
.. include:: ../exports/roles.include

.. _docker:

############
Docker image
############

A pre-compiled image of the |amlip| is not available at this stage.
However, there is a Dockerfile available to create your own Docker image `here <https://github.com/eProsima/AML-IP/tree/main/docker>`__.

This image is installed with an AML-IP that is able to run demo nodes.

Getting the Dockerfile
=======================

Download the AML-IP repository.

.. code-block:: bash

    git clone https://github.com/eProsima/AML-IP.git

Building the Docker image
=========================

Navigate to the Docker directory.

.. code-block:: bash

    cd amnlip/docker

Build the Docker image.

.. code-block:: bash

    docker build -t amlip --no-cache -f Dockerfile .

Using the Docker image
======================

Instructions to run an already built Docker image
-------------------------------------------------

.. code-block:: bash

    # Run docker image
    docker run --rm -it --net=host --ipc=host amlip

Instructions on how to build it
-------------------------------

.. code-block:: bash

    # Build docker image (from workspace where Dockerfile is)
    docker build --rm -t amlip -f Dockerfile .
    # use --no-cache argument to restart build

