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

namespace eprosima {
namespace amlip {
namespace node {
namespace test {

uint32_t PROCESS_DATA_TIME_MS = 100u;

} /* namespace test */
} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

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
        parent_id = parent_node.id();

        // Wait so status reader has time to process the data
        std::this_thread::sleep_for(std::chrono::milliseconds(node::test::PROCESS_DATA_TIME_MS));
    }

    // Give time for dropped data to arrive
    std::this_thread::sleep_for(std::chrono::milliseconds(node::test::PROCESS_DATA_TIME_MS));

    // Check the data that arrived
    bool parent_stopped = false;
    bool parent_dropped = false;

    for (const types::StatusDataType& data : data_arrived)
    {
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
 * Create a Status Node that stores status data
 * Create 2 Status nodes, one processing data with empty callback and other does not.
 * Check the stop and drop status messages arrive in both cases, while run and stop only in one case.
 */
TEST(StatusNodeTest, process_status_state)
{
    // Create Status Node
    node::StatusNode status_node("TestStatusNode");

    // Execute Status node and store the data that arrives
    types::AmlipIdDataType status_node_1_id;
    types::AmlipIdDataType status_node_2_id;
    std::vector<types::StatusDataType> data_arrived;
    status_node.process_status_async(
        [&data_arrived](const types::StatusDataType& data)
        {
            data_arrived.push_back(data);
        });

    {
        // Create Status Node 1 (process data)
        node::StatusNode status_node_1("StN1Test");
        status_node_1_id = status_node_1.id();

        // Create Status Node 2 (do not process data)
        node::StatusNode status_node_2("StN2Test");
        status_node_2_id = status_node_2.id();

        // Wait so status reader has time to process the data
        std::this_thread::sleep_for(std::chrono::milliseconds(node::test::PROCESS_DATA_TIME_MS));

        // Activate first Node
        status_node_1.process_status_async(
            [](const types::StatusDataType& data)
            {
                // Do nothing
            });

        // Wait so status reader has time to process the data
        std::this_thread::sleep_for(std::chrono::milliseconds(node::test::PROCESS_DATA_TIME_MS));

        // Deactivate first Node
        status_node_1.stop_processing();

        // Wait so status reader has time to process the data
        std::this_thread::sleep_for(std::chrono::milliseconds(node::test::PROCESS_DATA_TIME_MS));

        // Both nodes will be destroyed here
    }

    // Give time for dropped data to arrive
    std::this_thread::sleep_for(std::chrono::milliseconds(node::test::PROCESS_DATA_TIME_MS));

    // Check the data that arrived
    uint32_t node_1_stop = 0;
    uint32_t node_1_run = 0;
    uint32_t node_1_drop = 0;
    uint32_t node_2_stop = 0;
    uint32_t node_2_run = 0;
    uint32_t node_2_drop = 0;

    for (const types::StatusDataType& data : data_arrived)
    {
        if (data.id() == status_node_1_id)
        {
            // First Node
            ASSERT_EQ(types::NodeKind::STATUS, data.node_kind());
            if (data.state() == types::StateKind::STOPPED)
            {
                node_1_stop++;
            }
            else if (data.state() == types::StateKind::DROPPED)
            {
                node_1_drop++;
            }
            else if (data.state() == types::StateKind::RUNNING)
            {
                node_1_run++;
            }
        }
        else if (data.id() == status_node_2_id)
        {
            // First Node
            ASSERT_EQ(types::NodeKind::STATUS, data.node_kind());
            if (data.state() == types::StateKind::STOPPED)
            {
                node_2_stop++;
            }
            else if (data.state() == types::StateKind::DROPPED)
            {
                node_2_drop++;
            }
            else if (data.state() == types::StateKind::RUNNING)
            {
                node_2_run++;
            }
        }
    }

    ASSERT_EQ(node_1_drop, 1);
    ASSERT_EQ(node_1_run, 1);
    ASSERT_EQ(node_1_stop, 2);

    ASSERT_EQ(node_2_drop, 1);
    ASSERT_EQ(node_2_run, 0);
    ASSERT_EQ(node_2_stop, 1);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
