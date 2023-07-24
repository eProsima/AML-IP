.. include:: ../../exports/alias.include

.. _user_manual_nodes_model_sender:

#########################
Model Manager Sender Node
#########################

This kind of Node performs the passive (server) action of :ref:`user_manual_scenarios_collaborative_learning`.
This node sends statistics about the models it manages.
Then, waits for a Model request serialized as :ref:`user_manual_scenarios_collaborative_learning_model`.
Once received, a user-implemented callback (`fetch_model`) is executed, whose output should be the requested model in the form of a :ref:`user_manual_scenarios_collaborative_learning_solution`.


Example of Usage
================

Steps
-----

* Create the Id of the node.
* Create the statistics to be sent.
* Instantiate the ModelManagerSender Node creating an object of such class with the Id and statistics previously created.
* Start the execution of the node.
* Wait for a model request to arrive and be answered.
* Stop the execution of the node.


.. tabs::

    .. tab:: C++

        .. code-block:: cpp

            class CustomModelReplier : public eprosima::amlip::node::ModelReplier
            {
            public:

                CustomModelReplier(
                        const std::shared_ptr<eprosima::utils::event::BooleanWaitHandler>& waiter)
                    : waiter_(waiter)
                {
                    // Do nothing
                }

                virtual eprosima::amlip::types::ModelSolutionDataType fetch_model (
                        const eprosima::amlip::types::ModelDataType data) override
                {
                    logUser(AMLIPCPP_MANUAL_TEST, "Processing data: " << data << " . Processing data...");

                    // Create new solution from data here
                    eprosima::amlip::types::ModelSolutionDataType solution("MOBILENET V1");

                    logUser(AMLIPCPP_MANUAL_TEST, "Processed model: " << solution << " . Returning model...");

                    waiter_->open();

                    return solution;
                }

                std::shared_ptr<eprosima::utils::event::BooleanWaitHandler> waiter_;
            };

            // Create the Id of the node
            eprosima::amlip::types::AmlipIdDataType id({"ModelManagerSender"}, {66, 66, 66, 66});

            // Create statistics data
            eprosima::amlip::types::ModelStatisticsDataType statistics("ModelManagerSenderStatistics");
            statistics.data("hello world");

            // Create ModelManagerSender Node
            eprosima::amlip::node::ModelManagerSenderNode model_sender_node(id, statistics);

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

            class CustomModelReplier(ModelReplier):

                def fetch_model(
                        self,
                        model: ModelDataType) -> ModelSolutionDataType:
                    solution = ModelSolutionDataType(model.to_string().upper())
                    print(f'Model request received from client\n'
                        f' model: {model.to_string()}\n'
                        f' solution: {solution.to_string()}')

                    waiter.open()

                    return solution

            # Create statistics data
            statistics_data = ModelStatisticsDataType('ModelManagerSenderStatistics')
            statistics_data.set_data('hello world')

            # Create the Id of the node
            id = AmlipIdDataType('ModelManagerSender')
            id.set_id([66, 66, 66, 66])

            # Create  a new ModelManagerSender Node
            model_sender_node = ModelManagerSenderNode(
                id=id,
                statistics=statistics_data,
                domain=100)

            # Start execution
            model_sender_node.start(
                listener=CustomModelReplier())

            # Wait for the solution to be sent
            waiter.wait()

            # Stop execution
            model_sender_node.stop()
