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

#include <node/ComputingNode.hpp>
#include <node/MainNode.hpp>
#include <node/ParentNode.hpp>
#include <node/StatusNode.hpp>
#include <types/AmlipIdDataType.hpp>

using namespace eprosima::amlip;

/**
 * Create Parent Node
 */
TEST(NodeCreationTest, create_parent)
{
    node::ParentNode node("TestNode", types::NodeKind::UNDETERMINED);

    ASSERT_EQ(types::StateKind::STOPPED, node.current_state());
    ASSERT_EQ(types::NodeKind::UNDETERMINED, node.node_kind());
}

/**
 * Create Status Node
 */
TEST(NodeCreationTest, create_status)
{
    node::StatusNode node("TestNode");

    ASSERT_EQ(types::StateKind::STOPPED, node.current_state());
    ASSERT_EQ(types::NodeKind::STATUS, node.node_kind());
}

/**
 * Create Main Node
 */
TEST(NodeCreationTest, create_main)
{
    node::MainNode node("TestNode");

    ASSERT_EQ(types::StateKind::STOPPED, node.current_state());
    ASSERT_EQ(types::NodeKind::MAIN, node.node_kind());
}

/**
 * Create Computing Node
 */
TEST(NodeCreationTest, create_computing)
{
    node::ComputingNode node("TestNode");

    ASSERT_EQ(types::StateKind::STOPPED, node.current_state());
    ASSERT_EQ(types::NodeKind::COMPUTING, node.node_kind());
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
