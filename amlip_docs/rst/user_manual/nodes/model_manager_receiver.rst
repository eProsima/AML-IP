.. include:: ../../exports/alias.include

.. _user_manual_nodes_model_receiver:

###########################
Model Manager Receiver Node
###########################

This kind of Node performs the active (client) action of :ref:`user_manual_scenarios_collaborative_learning`.
This node receives statistics about models and sends a request if it is interested in a particular model.
Then, waits for a more complex and accurate model serialized as :ref:`user_manual_scenarios_collaborative_learning_solution`.


Example of Usage
================

Steps
-----

* Create the Id of the node.
* Create the data you want to request.
* Instantiate the ModelManagerReceiver Node creating an object of such class with the Id and data previously created.
* Start the execution of the node.
* Wait for a model reply.
* Stop the execution of the node.


.. tabs::

    .. tab:: C++

        .. code-block:: cpp

            // Create the Id of the node
            eprosima::amlip::types::AmlipIdDataType id({"ModelManagerSender"}, {66, 66, 66, 66});

            // Create the data
            eprosima::amlip::types::ModelDataType data("MobileNet V1");

            // Create ModelManagerReceiver Node
            eprosima::amlip::node::ModelManagerReceiverNode model_receiver_node(id, data);

            // Create waiter
            std::shared_ptr<eprosima::utils::event::BooleanWaitHandler> waiter =
                std::make_shared<eprosima::utils::event::BooleanWaitHandler>(false, true);

            // Create listener to process statistics and replies
            std::shared_ptr<CustomModelListener> listener =
                std::make_shared<CustomModelListener>(waiter);

            // Start the execution
            model_receiver_node.start(listener);

            // Wait the solution
            waiter->wait();

            // Stop the execution
            model_receiver_node.stop();

    .. tab:: Python

        .. code-block:: python

            # Create the data
            data = ModelDataType('MobileNet V1')

            # Create the Id of the node
            id = AmlipIdDataType('ModelManagerReceiver')
            id.set_id([66, 66, 66, 66])

            # Create  a new ModelManagerReceiver Node
            model_receiver_node = ModelManagerReceiverNode(
                id=id,
                data=data,
                domain=100)

            # Start the execution
            model_receiver_node.start(
                callback_statistics=statistics_received,
                callback_model=model_received)

            # Wait the solution
            waiter.wait()

            # Stop the execution
            model_receiver_node.stop()
