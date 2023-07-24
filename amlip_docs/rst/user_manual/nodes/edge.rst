.. include:: ../../exports/alias.include

.. _user_manual_nodes_edge:

#########
Edge Node
#########

This node is able to send data serialized as :ref:`user_manual_datatype_inference` and it receives an Inference as :ref:`user_manual_datatype_inference_solution`.

***********
Synchronous
***********

This node kind does require **active** interaction with the user to perform its action.
Once the data is sent, the thread must wait for the inference to arrive before sending another data.
Users can use method :code:`request_inference` to send new data.
The thread calling this method will wait until the whole process has finished and the *Inference* has arrived from the :ref:`user_manual_nodes_inference` in charge of this data.
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

************
Asynchronous
************

Users can use method :code:`request_inference` to send new data.
The thread calling this method must wait until the whole process has finished and the *Inference* has arrived from the :ref:`user_manual_nodes_inference` in charge of this data that will process it by the Listener or callback given, and return the Inference calculated in other thread.
By destroying the node every internal entity is correctly destroyed.

Steps
-----

* Instantiate the Async Edge Node creating an object of such class with a name, a listener or callback and a domain.
* Create a new :code:`InferenceDataType` from an array of bytes.
* Send a data synchronously calling :code:`request_inference`.
* Wait for the inference.

.. tabs::

    .. tab:: Python

        .. code-block:: python

            def inference_received(
                    inference,
                    task_id,
                    server_id):
                print(f'Data received from server: {server_id}\n'
                      f' with id: {task_id}\n'
                      f' inference: {inference.to_string()}')

            def main():
                # Create a new Async Edge Node
                node = AsyncEdgeNode(
                    "My_Async_Edge_Node",
                    listener=InferenceListenerLambda(inference_received),
                    domain=DOMAIN_ID)

                # Create new data to be executed remotely
                data = InferenceDataType("Some data as byte array serialized from a string or bytes")

                # Send data to a remote Inference Node and waits for the inference
                task_id = node.request_inference(data)

                # User must wait to receive solution.
                # Out of scope, the node will be destroyed,
                # and thus the solution will not arrive.
