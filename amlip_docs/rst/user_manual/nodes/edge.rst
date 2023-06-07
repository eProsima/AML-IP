.. include:: ../../exports/alias.include

.. _user_manual_nodes_edge:

#########
Edge Node
#########

This node is able to send data serialized as :ref:`user_manual_datatype_inference` and it receives an Inference as :ref:`user_manual_datatype_inference_solution`.

.. warning::

    The use of an Edge node must be synchronous.
    This means that once the data is sent, the thread must wait for the inference to arrive before sending another data.
    Asynchronous methods are available on the :ref:`user_manual_nodes_async_edge`.

Example of Usage
================

This node kind does require **active** interaction with the user to perform its action.
Users can use method :code:`request_inference` to send new data.
The thread calling this method will wait until the whole process has finished and the *Inference* has arrived from the *Inference Node* in charge of this data.
By destroying the node every internal entity is correctly destroyed.

Steps
-----

* Instantiate the Edge Node creating an object of such class with a name.
* Create a new :code:`InferenceDataType` from an array of bytes.
* Send a data synchronously and wait for the inference by calling :code:`request_inference`.

.. tabs::

    .. tab:: C++

        .. code-block:: cpp

            // Create a new Edge Node
            auto node = eprosima::amlip::EdgeNode("My_Edge_Node");

            // Create new data to be executed remotely
            auto data = eprosima::amlip::InferenceDataType("Some data as byte array serialized from a string or bytes");

            // Send data to a remote Inference Node and waits for the inference
            // This could be called with an id as well, and it will return the server id that send the inference
            auto solution = node.request_inference(data);

    .. tab:: Python

        .. code-block:: python

            # Create a new Edge Node
            node = EdgeNode("My_Edge_Node")

            # Create new data to be executed remotely
            data = InferenceDataType("Some data as byte array serialized from a string or bytes")

            # Send data to a remote Inference Node and waits for the inference
            inference, server_id = node.request_inference(data)
