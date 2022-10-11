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

#include <node/ParentNode.hpp>
#include <types/AmlipIdDataType.hpp>

namespace eprosima {
namespace amlip {
namespace node {
namespace test {

template <class T>
void test_create_and_run_node(
    types::NodeKind node_kind)
{
    T node("TestNode");

    ASSERT_EQ(types::StateKind::STOPPED, node.current_state());

    ASSERT_EQ(node_kind, node.node_kind());

    node.run();

    ASSERT_EQ(types::StateKind::RUNNING, node.current_state());

    node.stop();

    ASSERT_EQ(types::StateKind::STOPPED, node.current_state());

}

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
    node::test::test_create_and_run_node<node::ParentNode>(types::NodeKind::UNDETERMINED);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
