.. include:: ../../exports/alias.include

.. _user_manual_nodes_async_inference:

###########################
Asynchronous Inference Node
###########################

This node waits for data serialized as :ref:`user_manual_datatype_inference`, and once received it calculate the inference whose output is the inference solution as :ref:`user_manual_datatype_inference_solution`.

Example of Usage
================

User can use method :code:`request_inference` from :ref:`user_manual_nodes_async_edge` to send new data.
The thread calling this method must wait until the whole process has finished and the *Inference* has arrived from the *Inference Node* in charge of this data that will process it by the Listener or callback given, and return the Inference calculated in other thread.
By destroying the node every internal entity is correctly destroyed.

Steps
-----

* Instantiate the Inference Node creating an object of such class with a name, a listener and a domain.
* Wait for the data by calling :code:`run`.

.. tabs::

    .. tab:: Python

        .. code-block:: python

            def process_inference(
                    dataset,
                    task_id,
                    client_id):
                # Do some code that calculates the inference
                return inference_solution

            def main():

                # Create a new Async Inference Node
                node = AsyncInferenceNode(
                    "My_Async_Inference_Node",
                    listener=InferenceReplierLambda(process_inference),
                    domain=DOMAIN_ID)

                inference_node.run()

                # Wait until Ctrl+C

                inference_node.stop()
