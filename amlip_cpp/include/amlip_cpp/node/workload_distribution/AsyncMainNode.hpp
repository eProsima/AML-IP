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
 * @file AsyncMainNode.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_NODE_WORKLOADDISTRIBUTION_ASYNCMAINNODE_HPP
#define AMLIPCPP__SRC_CPP_NODE_WORKLOADDISTRIBUTION_ASYNCMAINNODE_HPP

#include <memory>

#include <amlip_cpp/node/ParentNode.hpp>
#include <amlip_cpp/types/id/TaskId.hpp>
#include <amlip_cpp/types/job/JobDataType.hpp>
#include <amlip_cpp/types/job/JobSolutionDataType.hpp>

// Forward declaration of dds classes
namespace eprosima {
namespace amlip {
namespace dds {

template <typename Task, typename Solution>
class AsyncMultiServiceClient;

} /* namespace dds */

} /* namespace amlip */
} /* namespace eprosima */

namespace eprosima {
namespace amlip {
namespace node {

/**
 * @brief Object that listens for answers to task sent by a Main Node.
 *
 * This class is supposed to be implemented by a User and be given to a \c JobSolutionDataType , answers
 * from a Computing Node after sending a Task from this Main.
 * Every Solution message call \c solution_received with the message as argument.
 */
class SolutionListener
{
public:

    //! Default virtual dtor so it can be inherited.
    virtual ~SolutionListener() = default;

    /**
     * @brief Method that will be called with each Solution received
     *
     * @param solution Solution to a task previously sent.
     * @param task_id Id of the task which this solution answers.
     * @param server_id Id of the ComputingNode answering.
     */
    virtual void solution_received(
        std::unique_ptr<types::JobSolutionDataType> solution,
        const types::TaskId& task_id,
        const types::AmlipIdDataType& server_id) = 0;
};

/**
 * @brief This is a specialization of AML-IP Node that sends Job data and waits for the Solution.
 *
 * Main Nodes are the ones in charge of sending training data (Job) and collecting the solution to those (Solution).
 * Using \c request_job_solution it will send a Job to a Computing Node that is available, and will wait for
 * the Solution retrieved by such Node.
 */
class AsyncMainNode : public ParentNode
{
public:

    /**
     * @brief Construct a new Main Node object.
     *
     * @param name name of the Node (it is advisable to be unique, or at least representative).
     */
    AMLIP_CPP_DllAPI AsyncMainNode(
            const char* name,
            std::shared_ptr<SolutionListener> listener,
            uint32_t domain_id);

    AMLIP_CPP_DllAPI AsyncMainNode(
            const char* name,
            std::shared_ptr<SolutionListener> listener);

    /**
     * @brief Destroy the Main Node object and its internal DDS entities.
     *
     * @pre Cannot be destroyed while processing a job. Otherwise undefined behaviour.
     */
    AMLIP_CPP_DllAPI ~AsyncMainNode();

    /**
     * @brief Send a Job to any Computing Node available asynchronously.
     *
     * This call will use internal threads to send task and receive solution.
     * Thread calling this will be blocked as less as possible.
     * The solution will be received once it is ready by the Listener given in method \c solution_received
     * and related with this Job by \c TaskId returned.
     *
     * @param data [in] Job to send.
     *
     * @return Task id of the job sent, so it is possible to relation it with the solution
     * that will be received asynchronously by the Listener.
     *
     * @note ownership of ptrs arguments is done in such way that every data that enters the Node
     * is shared. This is because this way it is less restrictive than other ownerships.
     * However internal node will not copy this data, so efficiency is not lost because of this design decision.
     */
    AMLIP_CPP_DllAPI types::TaskId request_job_solution(
            std::shared_ptr<types::JobDataType> data);

protected:

    /**
     * @brief Reference to the MultiService Client that sends jobs.
     *
     * This is created from DDS Participant in ParentNode, and its destruction is handled by ParentNode.
     */
    std::shared_ptr<dds::AsyncMultiServiceClient<types::JobDataType, types::JobSolutionDataType>> job_client_;

    std::shared_ptr<SolutionListener> listener_;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_WORKLOADDISTRIBUTION_ASYNCMAINNODE_HPP */
