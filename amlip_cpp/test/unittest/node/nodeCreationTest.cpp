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

#include <amlip_cpp/node/workload_distribution/ComputingNode.hpp>
#include <amlip_cpp/node/workload_distribution/MainNode.hpp>
#include <amlip_cpp/node/ParentNode.hpp>
#include <amlip_cpp/node/StatusNode.hpp>
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

} /* namespace test */
} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

using namespace eprosima::amlip;

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

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
