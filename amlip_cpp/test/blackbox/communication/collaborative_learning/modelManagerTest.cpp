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

#include <cpp_utils/wait/BooleanWaitHandler.hpp>

#include <amlip_cpp/node/collaborative_learning/ModelManagerNode.hpp>

namespace test {

uint32_t NUMBER_OF_MESSAGES = 10;

class TestModelListener : public eprosima::amlip::node::ModelListener
{
public:
    TestModelListener(std::function<void(const eprosima::amlip::types::ModelDataType&)> callback)
        : callback_(callback)
    {
    }

    void model_received (const eprosima::amlip::types::ModelDataType& model) const override
    {
        callback_(model);
    }

protected:
    std::function<void(const eprosima::amlip::types::ModelDataType&)> callback_;
};

} /* namespace test */

using namespace eprosima::amlip;

/**
 * Launch 2 ModelManager nodes and let 1 publish a model and the other wait for a model.
 * Then node 2 publish a node and 1 waits for it. This will be executed N times.
 *
 * Models will be mocked with strings.
 */
TEST(modelManagerTest, ping_pong)
{
    // Create Manager 1
    node::ModelManagerNode manager_1("ManagerTest1");
    node::ModelManagerNode manager_2("ManagerTest2");

    eprosima::utils::event::BooleanWaitHandler wait_handler(false, true);
    std::atomic<unsigned int> models_received(0);

    // Managers always send same model in this test
    std::string model_str = "ModelMock and some other data";
    types::ModelDataType model(model_str);

    // Both managers will read model, set counter and open wait handler
    auto manager_lambda = [&wait_handler, &models_received, &model_str](const types::ModelDataType& model)
    {
        models_received++;
        ASSERT_EQ(model.to_string(), model_str);
        wait_handler.open();
    };

    manager_1.start_receiving(std::make_shared<test::TestModelListener>(manager_lambda));
    manager_2.start_receiving(std::make_shared<test::TestModelListener>(manager_lambda));

    // Ping pong N times
    for (unsigned int i=0u; i<test::NUMBER_OF_MESSAGES; i++)
    {
        // Publish model from 1
        manager_1.publish_model(model);

        // Wait until 2 have received it
        wait_handler.wait();
        wait_handler.close();

        // Send a model from 1
        manager_2.publish_model(model);

        // Wait until 1 has received it
        wait_handler.wait();
        wait_handler.close();
    }

    ASSERT_EQ(models_received, 2 * test::NUMBER_OF_MESSAGES);
}

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
