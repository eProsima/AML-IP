.. include:: ../../exports/alias.include

.. _user_manual_nodes_inference:

##############
Inference Node
##############

This node waits for data serialized as :ref:`user_manual_datatype_inference`, and once received it calculate the inference whose output is the inference solution as :ref:`user_manual_datatype_inference_solution` and send the result back.

***********
Synchronous
***********

This node kind does require **active** interaction with the user to perform its action.
This means that calling `process_inference` will wait for receiving data, and will only finish when the result is sent back.
User can use method :code:`request_inference` from :ref:`user_manual_nodes_edge` to send new data.
The thread calling this method will wait until the whole process has finished and the *Inference* has arrived from the *Inference Node* in charge of this data.
By destroying the node every internal entity is correctly destroyed.

Steps
-----

* Instantiate the Inference Node creating an object of such class with a name.
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
The thread calling this method must wait until the whole process has finished and the *Inference* has arrived from the *Inference Node* in charge of this data that will process it by the Listener or callback given, and return the Inference calculated in other thread.
By destroying the node every internal entity is correctly destroyed.

Steps
-----

* Instantiate the Async Inference Node creating an object of such class with a name, a listener or callback and a domain.
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
