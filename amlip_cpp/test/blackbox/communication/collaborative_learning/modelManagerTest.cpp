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

#include <fstream>

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
            const std::shared_ptr<eprosima::utils::event::BooleanWaitHandler>& waiter)
        : waiter_(waiter)
    {
    }

    virtual bool statistics_received (
            const eprosima::amlip::types::ModelStatisticsDataType statistics) override
    {
        if (strcmp(statistics.name().c_str(), "v2") == 0)
        {
            std::string data;

            std::ifstream file("resources/el_quijote.txt");
            if (file.is_open())
            {
                data = std::string((std::istreambuf_iterator<char>(file)), (std::istreambuf_iterator<char>()));
            }
            else
            {
                throw std::runtime_error("Failed to open file: resources/el_quijote.txt");
            }

            EXPECT_EQ(statistics.to_string(), data);
            EXPECT_EQ(statistics.data_size(), data.length());

            waiter_->open();

            return false;
        }
        else
        {
            // Decide if we want the model based on the statistics received
            return true;
        }

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

        eprosima::amlip::types::ModelReplyDataType solution;
        if (strcmp(data.to_string().c_str(), "MobileNet V1") == 0)
        {
            solution = eprosima::amlip::types::ModelReplyDataType("MOBILENET V1");
        }
        else if (strcmp(data.to_string().c_str(), "MobileNet V2") == 0)
        {
            solution = eprosima::amlip::types::ModelReplyDataType("MOBILENET V2");
        }
        else
        {
            solution = eprosima::amlip::types::ModelReplyDataType("Do not have this model :,(");
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
    {

        // Managers always send same model in this test

        // Create ModelManagerReceiver Node
        eprosima::amlip::types::AmlipIdDataType id_receiver("ModelManagerReceiver");
        // NOTE: this data must be created before nodes or nodes must stop before this is destroyed
        eprosima::amlip::types::ModelRequestDataType data("MobileNet V1");
        eprosima::amlip::node::ModelManagerReceiverNode model_receiver_node(id_receiver, data);

        logUser(AMLIPCPP_MANUAL_TEST, "Node created: " << model_receiver_node << ". Creating model...");

        // Create ModelManagerSender Node
        eprosima::amlip::types::AmlipIdDataType id_sender("ModelManagerSender");
        eprosima::amlip::node::ModelManagerSenderNode model_sender_node(id_sender);

        // Create statistics data
        std::string data_str = "hello world";
        model_sender_node.publish_statistics("v1", data_str);

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
    logUser(AMLIPCPP_MANUAL_TEST, "Finishing test...");

}

/**
 * Launch 1 ModelManagerReceiverNode and 1 ModelManagerSenderNode that publish long statistics.
 *
 */
TEST(modelManagerTest, long_string_statistics)
{
    // Activate log
    eprosima::utils::Log::SetVerbosity(eprosima::utils::Log::Kind::Info);
    {

        // Managers always send same model in this test

        // Create ModelManagerReceiver Node
        eprosima::amlip::types::AmlipIdDataType id_receiver("ModelManagerReceiver");
        // NOTE: this data must be created before nodes or nodes must stop before this is destroyed
        eprosima::amlip::types::ModelRequestDataType data("MobileNet V2");
        eprosima::amlip::node::ModelManagerReceiverNode model_receiver_node(id_receiver, data);

        logUser(AMLIPCPP_MANUAL_TEST, "Node receiver created: " << model_receiver_node << ". Creating sender...");

        // Create ModelManagerSender Node
        eprosima::amlip::types::AmlipIdDataType id_sender("ModelManagerSender");
        eprosima::amlip::node::ModelManagerSenderNode model_sender_node(id_sender);

        logUser(AMLIPCPP_MANUAL_TEST, "Node sender created: " << model_sender_node << ".");


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

        // Create statistics data
        std::string data_str;

        std::ifstream file("resources/el_quijote.txt");
        if (file.is_open())
        {
            data_str = std::string((std::istreambuf_iterator<char>(file)), (std::istreambuf_iterator<char>()));
        }
        else
        {
            throw std::runtime_error("Failed to open file: resources/el_quijote.txt");
        }

        model_sender_node.publish_statistics("v2", data_str);

        // Wait solution
        waiter->wait();

        // Stop nodes
        model_receiver_node.stop();
        model_sender_node.stop();

    }
    logUser(AMLIPCPP_MANUAL_TEST, "Finishing test...");

}

/**
 * Launch 1 ModelManagerReceiverNode and 1 ModelManagerSenderNode that publish long statistics.
 *
 */
TEST(modelManagerTest, long_vector_statistics)
{
    // Activate log
    eprosima::utils::Log::SetVerbosity(eprosima::utils::Log::Kind::Info);
    {

        // Managers always send same model in this test

        // Create ModelManagerReceiver Node
        eprosima::amlip::types::AmlipIdDataType id_receiver("ModelManagerReceiver");
        // NOTE: this data must be created before nodes or nodes must stop before this is destroyed
        eprosima::amlip::types::ModelRequestDataType data("MobileNet V2");
        eprosima::amlip::node::ModelManagerReceiverNode model_receiver_node(id_receiver, data);

        logUser(AMLIPCPP_MANUAL_TEST, "Node receiver created: " << model_receiver_node << ". Creating sender...");

        // Create ModelManagerSender Node
        eprosima::amlip::types::AmlipIdDataType id_sender("ModelManagerSender");
        eprosima::amlip::node::ModelManagerSenderNode model_sender_node(id_sender);

        logUser(AMLIPCPP_MANUAL_TEST, "Node sender created: " << model_sender_node << ".");


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

        // Create statistics data
        std::string data_str;

        std::ifstream file("resources/el_quijote.txt");
        if (file.is_open())
        {
            data_str = std::string((std::istreambuf_iterator<char>(file)), (std::istreambuf_iterator<char>()));
        }
        else
        {
            throw std::runtime_error("Failed to open file: resources/el_quijote.txt");
        }

        std::vector<eprosima::amlip::types::ByteType> data_vector;

        for (char c : data_str)
        {
            data_vector.push_back(static_cast<eprosima::amlip::types::ByteType>(c));
        }

        model_sender_node.publish_statistics("v2", data_vector);

        // Wait solution
        waiter->wait();

        // Stop nodes
        model_receiver_node.stop();
        model_sender_node.stop();

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
