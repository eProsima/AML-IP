.. include:: ../../exports/alias.include

.. _user_manual_nodes_model_receiver:

###########################
Model Manager Receiver Node
###########################

This kind of Node performs the active (client) action of :ref:`user_manual_scenarios_collaborative_learning`.
This node receives statistics about models and sends a request if it is interested in a particular one.
Then, waits for the arrival of the requested model, serialized as :ref:`user_manual_scenarios_collaborative_learning_model_reply`.


Example of Usage
================

Steps
-----

* Create the Id of the node.
* Create the data to request.
* Instantiate the ModelManagerReceiver Node creating an object of such class with the Id and data previously created.
* Start the execution of the node.
* Wait for statistics.
* Request the model.
* Stop the execution of the node.


.. tabs::

    .. tab:: C++

        .. code-block:: cpp

            #include <cpp_utils/wait/BooleanWaitHandler.hpp>

            #include <amlip_cpp/types/id/AmlipIdDataType.hpp>
            #include <amlip_cpp/types/model/ModelRequestDataType.hpp>
            #include <amlip_cpp/types/model/ModelReplyDataType.hpp>
            #include <amlip_cpp/types/model/ModelStatisticsDataType.hpp>

            #include <amlip_cpp/node/collaborative_learning/ModelManagerReceiverNode.hpp>


            class CustomModelListener : public eprosima::amlip::node::ModelListener
            {
            public:

                CustomModelListener(
                        const std::shared_ptr<eprosima::utils::event::BooleanWaitHandler>& waiter)
                    : waiter_(waiter)
                {
                    // Do nothing
                }

                virtual void statistics_received (
                        const eprosima::amlip::types::ModelStatisticsDataType statistics) override
                {
                    // Save the server id
                    server_id = statistics.server_id();

                    waiter_->open();

                    // Always request model
                    return true;
                }

                virtual bool model_received (
                        const eprosima::amlip::types::ModelReplyDataType model) override
                {
                    std::cout << "Model received: " << model << " ." << std::endl;

                    return true;
                }

                std::shared_ptr<eprosima::utils::event::BooleanWaitHandler> waiter_;
                eprosima::amlip::types::AmlipIdDataType server_id;
            };

            // Create the Id of the node
            eprosima::amlip::types::AmlipIdDataType id({"ModelManagerSender"}, {10, 20, 30, 40});

            // Create the data
            eprosima::amlip::types::ModelRequestDataType data("MobileNet V1");

            // Create ModelManagerReceiver Node
            eprosima::amlip::node::ModelManagerReceiverNode model_receiver_node(id, data);

            // Create waiter to wait for statistics
            std::shared_ptr<eprosima::utils::event::BooleanWaitHandler> waiter =
                std::make_shared<eprosima::utils::event::BooleanWaitHandler>(false, true);

            // Create listener to process statistics and replies
            std::shared_ptr<CustomModelListener> listener =
                std::make_shared<CustomModelListener>(waiter);

            // Start execution
            model_receiver_node.start(listener);

            // Wait for statistics
            waiter->wait();

            // ... do something ...

            // Request the model
            model_receiver_node.request_model(listener->server_id);

            // Stop execution
            model_receiver_node.stop();

    .. tab:: Python

        .. code-block:: python

            from py_utils.wait.BooleanWaitHandler import BooleanWaitHandler

            from amlip_py.types.AmlipIdDataType import AmlipIdDataType
            from amlip_py.types.ModelRequestDataType import ModelRequestDataType
            from amlip_py.types.ModelReplyDataType import ModelReplyDataType
            from amlip_py.types.ModelStatisticsDataType import ModelStatisticsDataType

            from amlip_py.node.ModelManagerReceiverNode import ModelManagerReceiverNode, ModelListener

            # Variable to wait for the statistics
            waiter = BooleanWaitHandler(True, False)

            class CustomModelListener(ModelListener):

                def statistics_received(
                        self,
                        statistics: ModelStatisticsDataType) -> bool:

                    waiter.open()

                    return True

                def model_received(
                        self,
                        model: ModelReplyDataType) -> bool:
                    print(f'Model reply received from server\n'
                          f' solution: {model.to_string()}')

                    return True

            # Create the data
            data = ModelRequestDataType('MobileNet V1')

            # Create the Id of the node
            id = AmlipIdDataType('ModelManagerReceiver')
            id.set_id([10, 20, 30, 40])

            # Create a new ModelManagerReceiver Node
            model_receiver_node = ModelManagerReceiverNode(
                id=id,
                data=data,
                domain=100)

            # Start execution
            model_receiver_node.start(
                listener=CustomModelListener())

            # Wait statistics
            waiter.wait()

            # ... do something ...

            # Request the model
            model_receiver_node.request_model(model_receiver_node.listener_.server_id)

            # Stop execution
            model_receiver_node.stop()
