.. include:: ../../exports/alias.include

.. _user_manual_nodes_status:

.. |status| replace:: *Status*

###########
Status Node
###########

This kind of node :term:`Subscribe` to |status| :term:`Topic`.
Thus it receives every |status| data from all the other :term:`Nodes <Node>` in the network.
This node is executed with a *function* associated that will be executed with each message received.

Example of Usage
================

Instantiate the Status Node by creating an object of such class giving it a name.
In order to start processing status of the network, call :code:`process_status_async`.
To stop processing data, call :code:`stop_processing`.
By destroying the node every internal entity is correctly destroyed (if running, it stops automatically).

.. tabs::

    .. tab:: C++

        .. code-block:: bash

            // Create a new Status Node
            auto node = StatusNode("My Status Node");

            // Process arrival data by printing it in stdout (defined by std::function)
            node.process_status_async(
                []( const types::StatusDataType& status ){ std::cout << status << std::endl; });

    .. tab:: Python

        .. code-block:: python

            # Create a new Status Node
            node = StatusNode("My Status Node");

            # Process arrival data by printing it in stdout (defined by lambda)
            node.process_status_async(
                callback=lambda status: print(f'{status}'))
