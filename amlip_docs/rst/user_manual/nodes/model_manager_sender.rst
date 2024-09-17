.. include:: ../../exports/alias.include

.. _user_manual_nodes_model_sender:

#########################
Model Manager Sender Node
#########################

This kind of Node performs the passive (server) action of :ref:`user_manual_scenarios_collaborative_learning`.
This node sends statistics about the models it manages.
Then, waits for a Model request serialized as :ref:`user_manual_scenarios_collaborative_learning_model_request`.
Once received, a user-implemented callback (`fetch_model`) is executed, whose output should be the requested model in the form of a :ref:`user_manual_scenarios_collaborative_learning_model_reply`.


Example of Usage
================

Steps
-----

* Create the Id of the node.
* Create the statistics to be sent.
* Instantiate the ModelManagerSender Node by creating an object of this class with the previously created Id and statistics.
* Start the execution of the node.
* Wait for a model request to arrive and be answered.
* Stop the execution of the node.


.. tabs::

    .. tab:: C++

        .. code-block:: cpp

            // Include the required headers
            #include <cpp_utils/wait/BooleanWaitHandler.hpp>

            #include <amlip_cpp/types/id/AmlipIdDataType.hpp>
            #include <amlip_cpp/types/model/ModelRequestDataType.hpp>
            #include <amlip_cpp/types/model/ModelReplyDataType.hpp>
            #include <amlip_cpp/types/model/ModelStatisticsDataType.hpp>

            #include <amlip_cpp/node/collaborative_learning/ModelManagerSenderNode.hpp>


            class CustomModelReplier : public eprosima::amlip::node::ModelReplier
            {
            public:

                CustomModelReplier(
                        const std::shared_ptr<eprosima::utils::event::BooleanWaitHandler>& waiter)
                    : waiter_(waiter)
                {
                    // Do nothing
                }

                virtual eprosima::amlip::types::ModelReplyDataType fetch_model (
                        const eprosima::amlip::types::ModelRequestDataType data) override
                {
                    std::cout << "Processing data: " << data << " . Processing data..." << std::endl;

                    // Create new solution from data here
                    eprosima::amlip::types::ModelReplyDataType solution("MOBILENET V1");

                    std::cout << "Processed model: " << solution << " . Returning model..." << std::endl;

                    waiter_->open();

                    return solution;
                }

                std::shared_ptr<eprosima::utils::event::BooleanWaitHandler> waiter_;
            };

            // Create the Id of the node
            eprosima::amlip::types::AmlipIdDataType id({"ModelManagerSender"}, {66, 66, 66, 66});

            // Create ModelManagerSender Node
            eprosima::amlip::node::ModelManagerSenderNode model_sender_node(id);

            // Create statistics data
            std::string data = "hello world";
            model_sender_node.publish_statistics("v0", data);

            // Create waiter
            std::shared_ptr<eprosima::utils::event::BooleanWaitHandler> waiter =
                std::make_shared<eprosima::utils::event::BooleanWaitHandler>(false, true);

            // Create listener to process requests and return replies
            std::shared_ptr<CustomModelReplier> replier =
                std::make_shared<CustomModelReplier>(waiter);

            // Start execution
            model_sender_node.start(replier);

            // Wait for the solution to be sent
            waiter->wait();

            // Stop execution
            model_sender_node.stop();

    .. tab:: Python

        .. code-block:: python

            from py_utils.wait.BooleanWaitHandler import BooleanWaitHandler

            from amlip_py.types.AmlipIdDataType import AmlipIdDataType
            from amlip_py.types.ModelRequestDataType import ModelRequestDataType
            from amlip_py.types.ModelReplyDataType import ModelReplyDataType
            from amlip_py.types.ModelStatisticsDataType import ModelStatisticsDataType

            from amlip_py.node.ModelManagerSenderNode import ModelManagerSenderNode, ModelReplier


            class CustomModelReplier(ModelReplier):

                def fetch_model(
                        self,
                        model: ModelRequestDataType) -> ModelReplyDataType:
                    solution = ModelReplyDataType(model.to_string().upper())
                    print(f'Model request received from client\n'
                        f' model: {model.to_string()}\n'
                        f' solution: {solution.to_string()}')

                    waiter.open()

                    return solution

            # Create the Id of the node
            id = AmlipIdDataType('ModelManagerSender')
            id.set_id([66, 66, 66, 66])

            # Create  a new ModelManagerSender Node
            model_sender_node = ModelManagerSenderNode(
                id=id,
                domain=100)

            model_sender_node.publish_statistics(
                'ModelManagerSenderStatistics',
                'hello world')

            # Start execution
            model_sender_node.start(
                listener=CustomModelReplier())

            # Wait for the solution to be sent
            waiter.wait()

            # Stop execution
            model_sender_node.stop()
