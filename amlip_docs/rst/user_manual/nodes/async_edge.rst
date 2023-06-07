.. include:: ../../exports/alias.include

.. _user_manual_nodes_async_edge:

######################
Asynchronous Edge Node
######################

This node is able to send data serialized as :ref:`user_manual_datatype_inference` and it receives an Inference as :ref:`user_manual_datatype_inference_solution`.

Example of Usage
================

.. note::
    This page is under maintenance and will be updated soon.

Steps
-----

* Create a :code:`BooleanWaitHandler` to wait for the inference.
* Instantiate the Edge Node creating an object of such class with a name, a listener and a domain.
* Create a new :code:`InferenceDataType` from an array of bytes.
* Send a data synchronously calling :code:`request_inference`.
* Wait for the inference by calling :code:`waiter.wait()`.

.. tabs::

    .. tab:: Python

        .. code-block:: python

            # Variable to wait to the inference
            waiter = BooleanWaitHandler(True, False)

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

                # Wait to received solution
                waiter.wait()
