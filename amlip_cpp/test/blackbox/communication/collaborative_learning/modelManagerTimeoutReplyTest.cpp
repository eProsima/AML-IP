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
#include <cpp_utils/wait/BooleanWaitHandler.hpp>

#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <amlip_cpp/node/collaborative_learning/ModelManagerSenderNode.hpp>
#include <amlip_cpp/node/collaborative_learning/ModelManagerReceiverNode.hpp>


namespace test {

class TestModelListener : public eprosima::amlip::node::ModelListener
{
public:

    TestModelListener(
            const std::shared_ptr<eprosima::utils::event::BooleanWaitHandler>& waiter_statistics)
        : waiter_statistics_(waiter_statistics)
    {
    }

    virtual void statistics_received (
            const eprosima::amlip::types::ModelStatisticsDataType statistics) override
    {
        logUser(AMLIPCPP_MANUAL_TEST, "Statistics received: " << statistics << " .");

        server_id = statistics.server_id();

        waiter_statistics_->open();

        logUser(AMLIPCPP_MANUAL_TEST, "Opening statistics waiter...");
    }

    virtual bool model_received (
            const eprosima::amlip::types::ModelReplyDataType model) override
    {
        logUser(AMLIPCPP_MANUAL_TEST, "Model received: " << model << " .");

        return true;
    }

    std::shared_ptr<eprosima::utils::event::BooleanWaitHandler> waiter_statistics_;

    eprosima::amlip::types::AmlipIdDataType server_id;
};

class TestModelReplier : public eprosima::amlip::node::ModelReplier
{
public:

    TestModelReplier()
    {
        // Do nothing
    }

    virtual eprosima::amlip::types::ModelReplyDataType fetch_model (
            const eprosima::amlip::types::ModelRequestDataType data) override
    {
        logUser(AMLIPCPP_MANUAL_TEST, "Processing data: " << data << " . Processing data...");

        eprosima::amlip::types::ModelReplyDataType solution("MOBILENET V1");

        logUser(AMLIPCPP_MANUAL_TEST, "Processed model: " << solution << " . Returning model...");

        return solution;
    }

};

} /* namespace test */

using namespace eprosima::amlip;

/**
 * Launch 1 ModelManagerReceiverNode and 2 ModelManagerSenderNode.
 *
 * The first Sender node publishes its statistics and then stops.
 * Simulating when a Sender sends its statistics and the Receiver requests
 * its model, but the Receiver has already terminated, so Receiver waits for
 * REPLY_TIMEOUT_ ms before continuing to listen to other statistics.
 *
 * The second Sender publishes its statistics and responds to the
 * request from the Receiver.
 *
 * This test assesses how the ModelManagerReceiverNodes handle scenarios
 * where the ModelManagerSenderNodes terminate unexpectedly.
 *
 * Models will be mocked with strings.
 */
TEST(modelManagerTimeoutReplyTest, ping_pong)
{
    // Activate log
    eprosima::utils::Log::SetVerbosity(eprosima::utils::Log::Kind::Info);
    {

        // Managers always send same model in this test

        // Create ModelManagerReceiver Node
        eprosima::amlip::types::AmlipIdDataType id_receiver("ModelManagerReceiver");
        // NOTE: this data must be created before nodes or nodes must stop before this is destroyed
        eprosima::amlip::types::ModelRequestDataType data("MobileNet V1");
        eprosima::amlip::node::ModelManagerReceiverNode model_receiver_node(id_receiver, data);

        logUser(AMLIPCPP_MANUAL_TEST, "Node created: " << model_receiver_node << ". Creating model...");

        // Create ModelManagerSender Nodes
        eprosima::amlip::types::AmlipIdDataType id_sender_1("ModelManagerSender_1");
        eprosima::amlip::node::ModelManagerSenderNode model_sender_node_1(id_sender_1);

        eprosima::amlip::types::AmlipIdDataType id_sender_2("ModelManagerSender_2");
        eprosima::amlip::node::ModelManagerSenderNode model_sender_node_2(id_sender_2);

        // Create statistics data
        std::string data_str_1 = "Hello world, I'm going to die.";
        model_sender_node_1.publish_statistics("v0", data_str_1);

        // Create waiter receiver
        std::shared_ptr<eprosima::utils::event::BooleanWaitHandler> wait_statistics =
                std::make_shared<eprosima::utils::event::BooleanWaitHandler>(false, true);

        // Create listener
        std::shared_ptr<test::TestModelListener> listener =
                std::make_shared<test::TestModelListener>(wait_statistics);

        std::shared_ptr<test::TestModelReplier> replier =
                std::make_shared<test::TestModelReplier>();

        // Statistics are sent due to transient local qos even if the node is stopped
        // but, since the inner RPCReader is disabled, no reply is sent
        model_sender_node_1.stop();

        // Start nodes
        model_receiver_node.start(listener);

        // Wait statistics
        wait_statistics->wait();
        logUser(AMLIPCPP_MANUAL_TEST, "Statistics received.");

        // do something...
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        // decide to request the model
        model_receiver_node.request_model(listener->server_id);

        wait_statistics->close(); // Close to wait for next statistics

        // Create statistics data
        std::string data_str_2 = "Hello world, I'm working.";
        model_sender_node_2.publish_statistics("v1", data_str_2);

        model_sender_node_2.start(replier);

        wait_statistics->wait();
        logUser(AMLIPCPP_MANUAL_TEST, "Statistics received. Requesting model...");

        // do something...
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        // decide to request the model
        model_receiver_node.request_model(listener->server_id);

        // Stop nodes
        model_receiver_node.stop();
        model_sender_node_2.stop();
    }
    logUser(AMLIPCPP_MANUAL_TEST, "Finishing test...");

}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
