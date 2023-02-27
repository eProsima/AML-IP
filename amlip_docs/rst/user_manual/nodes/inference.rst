.. include:: ../../exports/alias.include

.. |status| replace:: *Status*

.. _user_manual_nodes_inference:

##############
Inference Node
##############

This kind of Node performs the passive (server) action of :ref:`user_manual_scenarios_wan`.
This node waits for data serialized as :ref:`user_manual_scenarios_wan_inference`, and once received it calculate the inference whose output is the inference solution as :ref:`user_manual_scenarios_wan_inference_solution`.

.. warning::

    In the current release, the use of a Edge node must be synchronous.
    This means that once the data is sent, the thread must wait for the inference to arrive before sending another data.
    In future release asynchronous methods will be available.


Example of Usage
================

This node kind does require **active** interaction with the user to perform its action.
User can use method :code:`request_inference` to send new data.
The thread calling this method will wait until the whole process has finished and the *Inference* has arrived from
the *Inference Node* in charge of this data.
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
            auto engine_routine = []( const eprosima::amlip::types::InferenceDataType& ){
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

            def engine_routine():
                InferenceSolutionDataType inference;
                # Do some code that calculates the inference
                return inference

            # Wait for 1 task from any client and answer it with process_inference callback
            node.process_inference(callback=engine_routine)
