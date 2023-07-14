// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file model_manager_sender.cpp
 *
 */

#include <thread>

#include <cpp_utils/Log.hpp>
#include <cpp_utils/wait/BooleanWaitHandler.hpp>

#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <amlip_cpp/types/model/ModelDataType.hpp>
#include <amlip_cpp/types/model/ModelSolutionDataType.hpp>
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

int main(
        int argc,
        char** argv)
{
    // Activate log
    eprosima::utils::Log::SetVerbosity(eprosima::utils::Log::Kind::Info);

    logUser(AMLIPCPP_MANUAL_TEST,
            "Starting Manual Test Model Manager Sender Node execution. Creating Node...");

    {
        // Create statistics data
        eprosima::amlip::types::AmlipIdDataType id({"ModelManagerSender"}, {66, 66, 66, 66});

        // Create ModelManagerSender Node
        eprosima::amlip::node::ModelManagerSenderNode model_sender_node(id);

        // Create statistics data
        std::string data = "hello world";
        model_sender_node.update_statistics("v0", data);

        logUser(AMLIPCPP_MANUAL_TEST, "Node created: " << model_sender_node << ". Creating model...");

        // Create waiter
        std::shared_ptr<eprosima::utils::event::BooleanWaitHandler> waiter =
                std::make_shared<eprosima::utils::event::BooleanWaitHandler>(false, true);

        // Create listener
        std::shared_ptr<CustomModelReplier> replier =
                std::make_shared<CustomModelReplier>(waiter);
        model_sender_node.start(replier);

        // Wait for the solution to be sent
        waiter->wait();

        model_sender_node.stop();

        logUser(AMLIPCPP_MANUAL_TEST, "Server has sended model. Destroying entities...");
    }

    logUser(AMLIPCPP_MANUAL_TEST, "Finishing Manual Test Model Manager Sender Node execution.");

    return 0;
}
