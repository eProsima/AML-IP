// Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include <amlip_cpp/node/wan/AgentNode.hpp>
#include <amlip_cpp/node/wan/ClientNode.hpp>
#include <amlip_cpp/node/wan/ServerNode.hpp>
#include <amlip_cpp/node/wan/TurnNode.hpp>

#include <amlip_cpp/node/ParentNode.hpp>
#include <amlip_cpp/node/StatusNode.hpp>

#include <amlip_cpp/node/workload_distribution/ComputingNode.hpp>
#include <amlip_cpp/node/workload_distribution/MainNode.hpp>
#include <amlip_cpp/node/workload_distribution/AsyncComputingNode.hpp>
#include <amlip_cpp/node/workload_distribution/AsyncMainNode.hpp>

#include <amlip_cpp/node/EdgeNode.hpp>
#include <amlip_cpp/node/InferenceNode.hpp>
#include <amlip_cpp/node/AsyncEdgeNode.hpp>
#include <amlip_cpp/node/AsyncInferenceNode.hpp>

#include <amlip_cpp/node/collaborative_learning/ModelManagerSenderNode.hpp>
#include <amlip_cpp/node/collaborative_learning/ModelManagerReceiverNode.hpp>

#include <amlip_cpp/types/id/AmlipIdDataType.hpp>

namespace eprosima {
namespace amlip {
namespace node {
namespace test {

class DummyNode : public ParentNode
{
public:

    DummyNode(
            const std::string& name,
            types::NodeKind node_kind)
        : ParentNode(name, node_kind)
    {
    }

};

class DummyListener : public eprosima::amlip::node::SolutionListener,
    public eprosima::amlip::node::InferenceSolutionListener
{
public:

    DummyListener()
    {
    }

    virtual void solution_received(
            const eprosima::amlip::types::GenericDataType& solution,
            const eprosima::amlip::types::TaskId& task_id,
            const eprosima::amlip::types::AmlipIdDataType& server_id) override
    {
        logUser(
            AMLIPCPP_MANUAL_TEST,
            "Solution received for task : " << task_id
                                            << " answered from server : " << server_id
                                            << " . Solution : " << solution << " .");
    }

    virtual void inference_received(
            const eprosima::amlip::types::GenericDataType& solution,
            const eprosima::amlip::types::TaskId& task_id,
            const eprosima::amlip::types::AmlipIdDataType& server_id) override
    {
        logUser(
            AMLIPCPP_MANUAL_TEST,
            "Inference received for task : " << task_id
                                             << " answered from server : " << server_id
                                             << " . Inference : " << solution << " .");
    }

};

class DummyReplier : public eprosima::amlip::node::JobReplier, public eprosima::amlip::node::InferenceReplier
{
public:

    DummyReplier()
    {
    }

    virtual eprosima::amlip::types::GenericDataType process_job(
            const eprosima::amlip::types::GenericDataType& task,
            const eprosima::amlip::types::TaskId& task_id,
            const eprosima::amlip::types::AmlipIdDataType& client_id) override
    {
        eprosima::amlip::types::GenericDataType solution;
        logUser(
            AMLIPCPP_MANUAL_TEST,
            "Job: " << task_id
                    << " received from client : " << client_id
                    << " .");
        return solution;
    }

    virtual eprosima::amlip::types::GenericDataType process_inference(
            const eprosima::amlip::types::GenericDataType& task,
            const eprosima::amlip::types::TaskId& task_id,
            const eprosima::amlip::types::AmlipIdDataType& client_id) override
    {
        eprosima::amlip::types::GenericDataType solution;
        logUser(
            AMLIPCPP_MANUAL_TEST,
            "Info: " << task_id
                     << " received from client : " << client_id
                     << " .");
        return solution;
    }

};

} /* namespace test */
} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

using namespace eprosima::amlip;

/**
 * Create Client Node
 */
TEST(NodeCreationTest, create_client)
{
    auto connection_address = eprosima::ddspipe::participants::types::Address(
        12121,
        12121,
        "localhost",
        eprosima::ddspipe::participants::types::TransportProtocol::udp);

    node::agent::ClientNode node("TestNode", { connection_address });

    ASSERT_EQ(types::StateKind::running, node.current_state());
    ASSERT_EQ(types::NodeKind::agent, node.node_kind());
}

/**
 * Create Server Node
 */
TEST(NodeCreationTest, create_server)
{
    auto listening_address = eprosima::ddspipe::participants::types::Address(
        12121,
        12121,
        "localhost",
        eprosima::ddspipe::participants::types::TransportProtocol::udp);

    node::agent::ServerNode node("TestNode", { listening_address });

    ASSERT_EQ(types::StateKind::running, node.current_state());
    ASSERT_EQ(types::NodeKind::agent, node.node_kind());
}

/**
 * Create Turn Node
 */
TEST(NodeCreationTest, create_turn)
{
    auto listening_address = eprosima::ddspipe::participants::types::Address(
        12121,
        12121,
        "localhost",
        eprosima::ddspipe::participants::types::TransportProtocol::udp);

    node::agent::TurnNode node("TestNode", { listening_address });

    ASSERT_EQ(types::StateKind::running, node.current_state());
    ASSERT_EQ(types::NodeKind::agent, node.node_kind());
}

/**
 * Create Parent Node
 */
TEST(NodeCreationTest, create_parent)
{
    node::test::DummyNode node("TestNode", types::NodeKind::undetermined);

    ASSERT_EQ(types::StateKind::stopped, node.current_state());
    ASSERT_EQ(types::NodeKind::undetermined, node.node_kind());
}

/**
 * Create Status Node
 */
TEST(NodeCreationTest, create_status)
{
    node::StatusNode node("TestNode");

    ASSERT_EQ(types::StateKind::stopped, node.current_state());
    ASSERT_EQ(types::NodeKind::status, node.node_kind());
}

/**
 * Create Main Node
 */
TEST(NodeCreationTest, create_main)
{
    node::MainNode node("TestNode");

    ASSERT_EQ(types::StateKind::stopped, node.current_state());
    ASSERT_EQ(types::NodeKind::main, node.node_kind());
}

/**
 * Create Computing Node
 */
TEST(NodeCreationTest, create_computing)
{
    node::ComputingNode node("TestNode");

    ASSERT_EQ(types::StateKind::stopped, node.current_state());
    ASSERT_EQ(types::NodeKind::computing, node.node_kind());
}

/**
 * Create AsyncMain Node
 */
TEST(NodeCreationTest, create_async_main)
{
    {
        // Create listener
        std::shared_ptr<node::test::DummyListener> listener =
                std::make_shared<node::test::DummyListener>();

        // Create AsyncMain Node
        node::AsyncMainNode node("TestNode", listener);

        ASSERT_EQ(types::StateKind::running, node.current_state());
        ASSERT_EQ(types::NodeKind::main, node.node_kind());
    }
}

/**
 * Create AsyncComputing Node
 */
TEST(NodeCreationTest, create_async_computing)
{
    // Create listener
    std::shared_ptr<node::test::DummyReplier> listener =
            std::make_shared<node::test::DummyReplier>();

    // Create AsyncComputing Node
    node::AsyncComputingNode node("TestNode", listener);

    ASSERT_EQ(types::StateKind::stopped, node.current_state());
    ASSERT_EQ(types::NodeKind::computing, node.node_kind());
}

/**
 * Create Edge Node
 */
TEST(NodeCreationTest, create_edge)
{
    node::EdgeNode node("TestNode");

    ASSERT_EQ(types::StateKind::stopped, node.current_state());
    ASSERT_EQ(types::NodeKind::edge, node.node_kind());
}

/**
 * Create Inference Node
 */
TEST(NodeCreationTest, create_inference)
{
    node::InferenceNode node("TestNode");

    ASSERT_EQ(types::StateKind::stopped, node.current_state());
    ASSERT_EQ(types::NodeKind::inference, node.node_kind());
}

/**
 * Create AsyncEdge Node
 */
TEST(NodeCreationTest, create_async_edge)
{
    {
        // Create listener
        std::shared_ptr<node::test::DummyListener> listener =
                std::make_shared<node::test::DummyListener>()   ;

        // Create AsyncEdge Node
        node::AsyncEdgeNode node("TestNode", listener)  ;

        ASSERT_EQ(types::StateKind::running, node.current_state());
        ASSERT_EQ(types::NodeKind::edge, node.node_kind());
    }
}

/**
 * Create AsyncInference Node
 */
TEST(NodeCreationTest, create_async_inference)
{
    // Create listener
    std::shared_ptr<node::test::DummyReplier> listener =
            std::make_shared<node::test::DummyReplier>();

    // Create AsyncInference Node
    node::AsyncInferenceNode node("TestNode", listener);

    ASSERT_EQ(types::StateKind::stopped, node.current_state());
    ASSERT_EQ(types::NodeKind::inference, node.node_kind());
}

/**
 * Create ModelManagerSender Node
 */
TEST(NodeCreationTest, create_model_sender)
{
    node::ModelManagerSenderNode node("TestNode");

    ASSERT_EQ(types::StateKind::stopped, node.current_state());
    ASSERT_EQ(types::NodeKind::model_sender, node.node_kind());
}

/**
 * Create ModelManagerReceiver Node
 */
TEST(NodeCreationTest, create_model_receiver)
{
    node::ModelManagerReceiverNode node("TestNode");

    ASSERT_EQ(types::StateKind::stopped, node.current_state());
    ASSERT_EQ(types::NodeKind::model_receiver, node.node_kind());
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
