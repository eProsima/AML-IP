.. include:: ../../exports/alias.include

.. _user_manual_nodes_async_edge:

######################
Asynchronous Edge Node
######################

This node is able to send data serialized as :ref:`user_manual_datatype_inference` and it receives an Inference as :ref:`user_manual_datatype_inference_solution`.

Example of Usage
================

Users can use method :code:`request_inference` to send new data.
The thread calling this method must wait until the whole process has finished and the *Inference* has arrived from the :ref:`user_manual_nodes_async_inference` in charge of this data that will process it by the Listener or callback given, and return the Inference calculated in other thread.
By destroying the node every internal entity is correctly destroyed.

Steps
-----

* Instantiate the Edge Node creating an object of such class with a name, a listener and a domain.
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
                waiter.open()

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
