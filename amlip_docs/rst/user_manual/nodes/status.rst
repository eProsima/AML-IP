.. include:: ../../exports/alias.include

.. |status| replace:: *Status*

.. _user_manual_nodes_status:

###########
Status Node
###########

This kind of node :term:`Subscribe` to |status| :term:`Topic`.
Thus it receives every |status| data from all the other :term:`Nodes <Node>` in the network.
This node runs a *function* that will be executed with each message received.
This is the main agent of :ref:`user_manual_scenarios_status`.

Example of Usage
================

This node kind does require **few interaction** with the user once it is running.
User must start and stop this node as desired using methods :code:`process_status_async` and :code:`stop_processing`.
Also, user must yield a callback (function) that will be executed with every |status| message received.
By destroying the node it stops if running, and every internal entity is correctly destroyed.

Steps
-----

* Instantiate the Status Node creating an object of such class with a name.
* Start processing status data of the network calling :code:`process_status_async`.
* Stop processing data calling :code:`stop_processing`.

.. tabs::

    .. tab:: C++

        .. code-block:: cpp

            // Create a new Status Node
            auto node = eprosima::amlip::StatusNode("My_Status_Node");

            // Process arrival data by printing it in stdout (defined by std::function)
            node.process_status_async(
                []( const eprosima::amlip::types::StatusDataType& status ){ std::cout << status << std::endl; });

            // Do other cool things here

            // Stop processing data
            node.stop_processing()

    .. tab:: Python

        .. code-block:: python

            # Create a new Status Node
            node = StatusNode("My Status Node");

            # Process arrival data by printing it in stdout (defined by lambda)
            node.process_status_async(
                callback=lambda status: print(f'{status}'))

            # Do other cool things here

            # Stop processing data
            node.stop_processing()
