.. include:: ../../exports/alias.include

.. _user_manual_nodes_async_inference:

###########################
Asynchronous Inference Node
###########################

This node waits for data serialized as :ref:`user_manual_datatype_inference`, and once received it calculate the inference whose output is the inference solution as :ref:`user_manual_datatype_inference_solution`.

Example of Usage
================

.. note::
    This page is under maintenance and will be updated soon.

Steps
-----

* Instantiate the Inference Node creating an object of such class with a name, a listener and a domain.
* Wait for the data by calling :code:`run`.
* Return the inference as an :code:`InferenceSolutionDataType`.

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
