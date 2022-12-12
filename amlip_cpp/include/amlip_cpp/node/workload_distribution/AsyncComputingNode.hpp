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
 * @file AsyncComputingNode.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_NODE_WORKLOADDISTRIBUTION_ASYNCCOMPUTINGNODE_HPP
#define AMLIPCPP__SRC_CPP_NODE_WORKLOADDISTRIBUTION_ASYNCCOMPUTINGNODE_HPP

#include <functional>

#include <amlip_cpp/node/ParentNode.hpp>
#include <amlip_cpp/types/id/TaskId.hpp>
#include <amlip_cpp/types/job/JobDataType.hpp>
#include <amlip_cpp/types/job/JobSolutionDataType.hpp>

// Forward declaration of dds classes
namespace eprosima {
namespace amlip {
namespace dds {

template <typename Task, typename Solution>
class AsyncMultiServiceServer;

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

namespace eprosima {
namespace amlip {
namespace node {

/**
 * @brief Object that is called when a new Job data has been received.
 *
 * This class is supposed to be implemented by a User and be given to a \c ComputingNode in order to process a Job.
 * When a Job is received, \c process_job is called and it is expected to return a Solution for such job.
 */
class AMLIP_CPP_DllAPI JobListener
{
public:

    //! Default virtual dtor so it can be inherited.
    virtual ~JobListener() = default;

    /**
     * @brief Method that will be called with the Job message received to calculate an answer.
     *
     * @param job new Job message received.
     * @param task_id Id of the Task received.
     * @param client_id Id of the Client that sent this job.
     *
     * @return Solution to the \c job .
     */
    virtual types::JobSolutionDataType process_job (
            std::unique_ptr<types::JobDataType> job,
            const types::TaskId& task_id,
            const types::AmlipIdDataType& client_id) = 0;
};

/**
 * @brief This is a specialization of AML-IP Node that waits for Job data and retrieves a Solution.
 *
 * Computing Nodes are the ones in charge of receiving training data from a Main Node
 * Using \c process_job will wait for a Main Node to send it the data for a Job, and will process this Job by
 * the Listener or callback given, and return the Solution calculated.
 */
class AsyncComputingNode : public ParentNode
{
public:

    /**
     * @brief Construct a new Computing Node object.
     *
     * @param name name of the Node (it is advisable to be unique, or at least representative).
     */
    AMLIP_CPP_DllAPI AsyncComputingNode(
            const char* name,
            std::shared_ptr<JobListener> listener,
            uint32_t domain_id);

    AMLIP_CPP_DllAPI AsyncComputingNode(
            const char* name,
            std::shared_ptr<JobListener> listener);

    /**
     * @brief Destroy the Main Node object and its internal DDS entities.
     *
     * @pre Cannot be destroyed while processing a job. Otherwise undefined behaviour.
     */
    AMLIP_CPP_DllAPI ~AsyncComputingNode();

    /**
     * @brief Process Jobs asynchronously.
     *
     * This uses an internal thread that execute the whole multiservice process.
     * In order to calculate Jobs that are received, it uses \c process_job from Listener given.
     *
     * @throw if node is already running.
     */
    void run();

    /**
     * @brief Stop processing data.
     *
     * If not processing data, do nothing.
     */
    void stop();

protected:

    /**
     * @brief Reference to the MultiService Server that sends jobs.
     *
     * This is created from DDS Participant in ParentNode, and its destruction is handled by ParentNode.
     */
    std::shared_ptr<dds::AsyncMultiServiceServer<types::JobDataType, types::JobSolutionDataType>> job_server_;

    std::shared_ptr<JobListener> listener_;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_WORKLOADDISTRIBUTION_ASYNCCOMPUTINGNODE_HPP */
