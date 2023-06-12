.. include:: ../../exports/alias.include

.. _user_manual_nodes_async_inference:

###########################
Asynchronous Inference Node
###########################

This node waits for data serialized as :ref:`user_manual_datatype_inference`, and once received it calculate the inference whose output is the inference solution as :ref:`user_manual_datatype_inference_solution`.

Example of Usage
================

User can use method :code:`request_inference` to send new data.
The thread calling this method will wait until the whole process has finished and the *Inference* has arrived from
the *Inference Node* in charge of this data, and will process this data by the Listener or callback given, and return the Inference calculated.
By destroying the node every internal entity is correctly destroyed.

Steps
-----

* Instantiate the Inference Node creating an object of such class with a name, a listener and a domain.
* Wait for the data by calling :code:`run`.

.. tabs::

    .. tab:: Python

        .. code-block:: python

            def process_inference(
                    inference,
                    task_id,
                    client_id):
                inference_solution = InferenceSolutionDataType(inference.to_string().lower())
                print(f'Data received from client: {client_id}\n'
                      f' with id: {task_id}\n'
                      f' job: {inference.to_string()}\n'
                      f' inference: {inference_solution.to_string()}')
                return inference_solution

            def main():

                # Create a new Async Inference Node
                node = AsyncInferenceNode(
                    "My_Async_Inference_Node",
                    listener=InferenceReplierLambda(process_inference),
                    domain=DOMAIN_ID)

                inference_node.run()

                def handler(signum, frame):
                    pass
                signal.signal(signal.SIGINT, handler)
                signal.pause()

                inference_node.stop()
