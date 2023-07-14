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

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <cpp_utils/Log.hpp>
#include <cpp_utils/types/cast.hpp>
#include <cpp_utils/utils.hpp>
#include <cpp_utils/wait/BooleanWaitHandler.hpp>

#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <amlip_cpp/node/collaborative_learning/ModelManagerSenderNode.hpp>
#include <amlip_cpp/node/collaborative_learning/ModelManagerReceiverNode.hpp>


namespace test {

class TestModelListener : public eprosima::amlip::node::ModelListener
{
public:

    TestModelListener(
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
            const eprosima::amlip::types::ModelSolutionDataType model) override
    {
        logUser(AMLIPCPP_MANUAL_TEST, "Model received: " << model << " .");

        waiter_->open();

        return true;
    }

    std::shared_ptr<eprosima::utils::event::BooleanWaitHandler> waiter_;
};

class TestModelReplier : public eprosima::amlip::node::ModelReplier
{
public:

    TestModelReplier()
    {
        // Do nothing
    }

    virtual eprosima::amlip::types::ModelSolutionDataType fetch_model (
            const eprosima::amlip::types::ModelDataType data) override
    {
        logUser(AMLIPCPP_MANUAL_TEST, "Processing data: " << data << " . Processing data...");

        eprosima::amlip::types::ModelSolutionDataType solution;
        if (data.data() == "MobileNet V1")
        {
            solution = eprosima::amlip::types::ModelSolutionDataType("MOBILENET V1");
        }
        else if (data.data() == "MobileNet V2")
        {
            solution = eprosima::amlip::types::ModelSolutionDataType("MOBILENET V2");
        }
        else
        {
            solution = eprosima::amlip::types::ModelSolutionDataType("Do not have this model :,(");
        }
        logUser(AMLIPCPP_MANUAL_TEST, "Processed model: " << solution << " . Returning model...");

        return solution;
    }

};

} /* namespace test */

using namespace eprosima::amlip;

/**
 * Launch 1 ModelManagerReceiverNode and 1 ModelManagerSenderNode.
 *
 * Models will be mocked with strings.
 */
TEST(modelManagerTest, ping_pong)
{
    // Activate log
    eprosima::utils::Log::SetVerbosity(eprosima::utils::Log::Kind::Info);

    // Managers always send same model in this test
    // NOTE: this data must be created before nodes or nodes must stop before this is destroyed
    std::string model_str = "ModelMock and some other data";
    types::ModelDataType model(model_str);

    // Create ModelManagerReceiver Node
    eprosima::amlip::types::AmlipIdDataType id_receiver({"ModelManagerReceiver"}, {66, 66, 66, 66});
    eprosima::amlip::types::ModelDataType data("MobileNet V1");
    eprosima::amlip::node::ModelManagerReceiverNode model_receiver_node(id_receiver, data);

    logUser(AMLIPCPP_MANUAL_TEST, "Node created: " << model_receiver_node << ". Creating model...");

    // Create ModelManagerSender Node
    eprosima::amlip::types::AmlipIdDataType id_sender({"ModelManagerSender"}, {66, 66, 66, 66});
    eprosima::amlip::node::ModelManagerSenderNode model_sender_node(id_sender);
    std::string data_str = "hello world";

    // Create statistics data
    void* data_void = eprosima::utils::copy_to_void_ptr(eprosima::utils::cast_to_void_ptr(data_str.c_str()),
                    data_str.length());
    model_sender_node.update_statistics("v0", data_void, data_str.length());

    // Create waiter
    std::shared_ptr<eprosima::utils::event::BooleanWaitHandler> waiter =
            std::make_shared<eprosima::utils::event::BooleanWaitHandler>(false, true);

    // Create listener
    std::shared_ptr<test::TestModelListener> listener =
            std::make_shared<test::TestModelListener>(waiter);

    std::shared_ptr<test::TestModelReplier> replier =
            std::make_shared<test::TestModelReplier>();

    // Start nodes
    model_receiver_node.start(listener);
    model_sender_node.start(replier);

    // Wait solution
    waiter->wait();

    // Stop nodes
    model_receiver_node.stop();
    model_sender_node.stop();
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
