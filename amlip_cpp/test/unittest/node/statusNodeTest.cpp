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
#include <node/StatusNode.hpp>
#include <types/AmlipIdDataType.hpp>

using namespace eprosima::amlip;

/**
 * Create a Status node.
 * Process data with dummy(empty) callback.
 * Stop Node, close thread and destroy node correctly.
 * Check in each step that the state is correct
 *
 * CASES:
 * - stop before destruction
 * - no stop before destruction
 */
TEST(StatusNodeTest, run_and_stop)
{
    // stop before destruction
    {
        // Create Status Node
        node::StatusNode status_node("TestStatusNode");

        ASSERT_EQ(types::StateKind::STOPPED, status_node.current_state());

        status_node.process_status_async(
            [](const types::StatusDataType& data)
            {
                // Do nothing
            });

        ASSERT_EQ(types::StateKind::RUNNING, status_node.current_state());

        status_node.stop_processing();

        ASSERT_EQ(types::StateKind::STOPPED, status_node.current_state());
    }

    // no stop before destruction
    {
        // Create Status Node
        node::StatusNode status_node("TestStatusNode");

        ASSERT_EQ(types::StateKind::STOPPED, status_node.current_state());

        status_node.process_status_async(
            [](const types::StatusDataType& data)
            {
                // Do nothing
            });

        ASSERT_EQ(types::StateKind::RUNNING, status_node.current_state());
    }
}

/**
 * Create a Status node and process the data that arrives.
 * Create a Parent node and check the STOP and DROP data.
 */
TEST(StatusNodeTest, process_status_parent)
{
    // Create Status Node
    node::StatusNode status_node("TestStatusNode");

    // Execute Status node and store the data that arrives
    types::AmlipIdDataType parent_id;
    std::vector<types::StatusDataType> data_arrived;
    status_node.process_status_async(
        [&data_arrived](const types::StatusDataType& data)
        {
            data_arrived.push_back(data);
        });

    {
        // Create Parent Node to be destroyed afterwards
        node::ParentNode parent_node("TestParentNode", types::NodeKind::UNDETERMINED);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        parent_id = parent_node.id();
    }

    // Give time for data to arrive
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Check the data that arrived
    bool parent_stopped = false;
    bool parent_dropped = false;

    for (const types::StatusDataType& data : data_arrived)
    {
        std::cout << "Data arrived: " << data << std::endl;
        if (data.id() == parent_id)
        {
            ASSERT_EQ(types::NodeKind::UNDETERMINED, data.node_kind());
            if (data.state() == types::StateKind::STOPPED)
            {
                parent_stopped = true;
            }
            else if (data.state() == types::StateKind::DROPPED)
            {
                parent_dropped = true;
            }
        }
    }

    ASSERT_TRUE(parent_stopped);
    ASSERT_TRUE(parent_dropped);
}

/**
 * TODO
 */
TEST(StatusNodeTest, process_status_status)
{
    // TODO
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
