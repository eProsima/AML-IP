.. include:: ../../exports/alias.include

.. _user_manual_nodes_inference:

##############
Inference Node
##############

This node processes data serialized as :ref:`user_manual_datatype_inference`. Upon receiving the data, it computes the inference and produces an output in the form of :ref:`user_manual_datatype_inference_solution`, which is then sent back to the requester.

***********
Synchronous
***********

This node requires **active** user interaction to perform its tasks.
When calling :code:`process_inference`, the method will block and wait for incoming data, only completing once the result is sent back. Users can utilize the :code:`request_inference` method from :ref:`user_manual_nodes_edge` to submit new data.
The thread invoking this method will remain blocked until the entire process is completed and the *Inference* result is received from the responsible *Inference Node*.
By destroying the node every internal entity is correctly destroyed.

Steps
-----

* Instantiate the Inference Node creating an object of this class with a name.
* Wait for the data by calling :code:`process_inference`.
* Return the inference as an :code:`InferenceSolutionDataType`.

.. tabs::

    .. tab:: C++

        .. code-block:: cpp

            // Create a new Inference Node
            auto node = eprosima::amlip::InferenceNode("My_Inference_Node");

            // Create a callback to process data and return the inference
            auto engine_routine = []( const eprosima::amlip::types::InferenceDataType& dataset){
                eprosima::amlip::types::InferenceSolutionDataType inference;
                // Do some code that calculates the inference
                return inference;
            };

            // Wait for 1 task from any client and answer it with process_inference callback
            node.process_inference(engine_routine);

    .. tab:: Python

        .. code-block:: python

            # Create a new Inference Node
            node = InferenceNode("My_Inference_Node")

            def engine_routine(dataset):
                # Do some code that calculates the inference
                return InferenceSolutionDataType(inference_solution)

            # Wait for 1 task from any client and answer it with process_inference callback
            node.process_inference(callback=lambda inference: engine_routine(dataset))

************
Asynchronous
************

User can use method :code:`request_inference` from :ref:`user_manual_nodes_edge` to send new data.
Due to being asynchronous, multiple requests can be sent without waiting for the previous one to finish. The solution will be sent back to the user through the listener.
By destroying the node every internal entity is correctly destroyed.

Steps
-----

* Instantiate the Asynchronous Inference Node creating an object of this class with a name, a listener or callback and a domain.
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
                    domain=100)

                inference_node.run()

                # Wait until Ctrl+C

                inference_node.stop()
