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

/**
 * @file main.cpp
 *
 */

#include <thread>

#include <cpp_utils/Log.hpp>
#include <cpp_utils/wait/BooleanWaitHandler.hpp>

#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <amlip_cpp/node/workload_distribution/AsyncMainNode.hpp>

class CustomSolutionListener : public eprosima::amlip::node::SolutionListener
{
public:

    CustomSolutionListener(const std::shared_ptr<eprosima::utils::event::BooleanWaitHandler>& waiter)
        : waiter_(waiter)
    {
        // Do nothing
    }

    virtual void solution_received(
            const eprosima::amlip::types::JobSolutionDataType& solution,
            const eprosima::amlip::types::TaskId& task_id,
            const eprosima::amlip::types::AmlipIdDataType& server_id) override
    {
        logUser(
            AMLIPCPP_MANUAL_TEST,
            "Solution received for task : " << task_id
            << " answered from server : " << server_id
            << " . Solution : " << solution << " .");
        waiter_->open();
    }

    std::shared_ptr<eprosima::utils::event::BooleanWaitHandler> waiter_;
};

/*
 * The job in this example will be a string serialized to bytes.
 */
int main(
        int argc,
        char** argv)
{
    // initialize the random number generator
    srand (time(NULL));

    // Activate log
    // eprosima::utils::Log::SetVerbosity(eprosima::utils::Log::Kind::Info);

    logUser(AMLIPCPP_MANUAL_TEST, "Starting Manual Test Async Main Node execution. Creating Node...");

    {
        // Create waiter
        std::shared_ptr<eprosima::utils::event::BooleanWaitHandler> solution_waiter =
            std::make_shared<eprosima::utils::event::BooleanWaitHandler>(false, true);

        // Create listener
        std::shared_ptr<CustomSolutionListener> listener =
            std::make_shared<CustomSolutionListener>(solution_waiter);

        // Create Main Node
        eprosima::amlip::node::AsyncMainNode main_node("CppAsyncMainNode_Manual", listener);

        logUser(AMLIPCPP_MANUAL_TEST, "Node created: " << main_node << ". Creating job...");

        // Create job data
        std::string data_str = "<Job Data In String>";
        // The cast to char* is needed to avoid const in ptr
        eprosima::amlip::types::JobDataType job_data(static_cast<void*>(
                    const_cast<char*>(data_str.c_str())),
                data_str.size() + 1);

        logUser(AMLIPCPP_MANUAL_TEST, "Job data created with string: " << data_str << ". Sending request...");

        // Send job request
        auto task_id = main_node.request_job_solution(job_data);

        logUser(AMLIPCPP_MANUAL_TEST, "Task sent with id : " << task_id << ". Waiting solution...");

        // Wait solution
        solution_waiter->wait();

        logUser(AMLIPCPP_MANUAL_TEST, "Solution received. Destroying entities...");
    }

    logUser(AMLIPCPP_MANUAL_TEST, "Finishing Manual Test Async Main Node execution.");

    return 0;
}
