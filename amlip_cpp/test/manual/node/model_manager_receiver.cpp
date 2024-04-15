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
 * @file model_manager_receiver.cpp
 *
 */

#include <thread>

#include <cpp_utils/Log.hpp>
#include <cpp_utils/wait/BooleanWaitHandler.hpp>

#include <amlip_cpp/node/collaborative_learning/ModelManagerReceiverNode.hpp>
#include <amlip_cpp/types/id/AmlipIdDataType.hpp>


class CustomModelListener : public eprosima::amlip::node::ModelListener
{
public:

    CustomModelListener(
            const std::shared_ptr<eprosima::utils::event::BooleanWaitHandler>& waiter)
        : waiter_(waiter)
    {
    }

    virtual bool statistics_received (
            const eprosima::amlip::types::ModelStatisticsDataType statistics) override
    {
        // Decide if we want the model based on the statistics received
        return true;
    }

    virtual bool model_received (
            const eprosima::amlip::types::ModelReplyDataType model) override
    {
        logUser(AMLIPCPP_MANUAL_TEST, "Model received: " << model << " .");

        waiter_->open();

        return true;
    }

    std::shared_ptr<eprosima::utils::event::BooleanWaitHandler> waiter_;
};

int main(
        int argc,
        char** argv)
{
    // Activate log
    //     eprosima::utils::Log::SetVerbosity(eprosima::utils::Log::Kind::Info);

    logUser(AMLIPCPP_MANUAL_TEST,
            "Starting Manual Test Model Manager Receiver Node execution. Creating Node...");

    {
        // Create ModelManagerReceiver Node
        eprosima::amlip::types::AmlipIdDataType id({"ModelManagerReceiver"}, {66, 66, 66, 66});
        eprosima::amlip::types::ModelRequestDataType data("MobileNet V1");
        eprosima::amlip::node::ModelManagerReceiverNode model_receiver_node(id, data);

        logUser(AMLIPCPP_MANUAL_TEST, "Node created: " << model_receiver_node << ". Creating model...");

        // Create waiter
        std::shared_ptr<eprosima::utils::event::BooleanWaitHandler> waiter =
                std::make_shared<eprosima::utils::event::BooleanWaitHandler>(false, true);

        // Create listener
        std::shared_ptr<CustomModelListener> listener =
                std::make_shared<CustomModelListener>(waiter);
        model_receiver_node.start(listener);

        // Wait solution
        waiter->wait();

        model_receiver_node.stop();

        logUser(AMLIPCPP_MANUAL_TEST, "Client has received model. Destroying entities...");
    }

    logUser(AMLIPCPP_MANUAL_TEST, "Finishing Manual Test Model Manager Receiver Node execution.");

    return 0;
}
