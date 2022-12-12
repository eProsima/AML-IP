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
 * @file MainNode.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_NODE_MAINNODE_HPP
#define AMLIPCPP__SRC_CPP_NODE_MAINNODE_HPP

#include <functional>

#include <amlip_cpp/node/ParentNode.hpp>
#include <amlip_cpp/types/job/JobDataType.hpp>
#include <amlip_cpp/types/job/JobSolutionDataType.hpp>

// Forward declaration of dds classes
namespace eprosima {
namespace amlip {
namespace dds {

template <typename Task, typename Solution>
class MultiServiceClient;

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

namespace eprosima {
namespace amlip {
namespace node {

/**
 * @brief This is a specialization of AML-IP Node that sends Job data and waits for the Solution.
 *
 * Main Nodes are the ones in charge of sending training data (Job) and collecting the solution to those (Solution).
 * Using \c request_job_solution it will send a Job to a Computing Node that is available, and will wait for
 * the Solution retrieved by such Node.
 *
 * @todo implement an asynchronous request_job_solution method.
 *
 * @warning Not Thread Safe (yet) (TODO)
 */
class MainNode : public ParentNode
{
public:

    AMLIP_CPP_DllAPI MainNode(
            const char* name,
            uint32_t domain_id);

    /**
     * @brief Construct a new Main Node object.
     *
     * @param name name of the Node (it is advisable to be unique, or at least representative).
     */
    AMLIP_CPP_DllAPI MainNode(
            const char* name);

    //! Same as previous ctor but with a string argument.
    AMLIP_CPP_DllAPI MainNode(
            const std::string& name);

    /**
     * @brief Destroy the Main Node object and its internal DDS entities.
     *
     * @pre Cannot be destroyed while processing a job. Otherwise undefined behaviour.
     */
    AMLIP_CPP_DllAPI ~MainNode();

    /**
     * @brief Send a Job to any Computing Node available and wait for the solution.
     *
     * @param data [in] Job to send.
     *
     * @attention this method is synchronous and will not finish until the job has been solved.
     *
     * @todo asynchronous mode
     *
     * @return Solution of the Job
     */
    AMLIP_CPP_DllAPI types::JobSolutionDataType request_job_solution(
            const types::JobDataType& data);

    /**
     * @brief Send a Job to any Computing Node available and wait for the solution, getting the Id of the solver.
     *
     * @param data [in] Job to send.
     * @param server [out] Id of the Node that has answered the Job.
     *
     * @attention this method is synchronous and will not finish until the job has been solved.
     *
     * @todo asynchronous mode
     *
     * @return Solution of the Job
     */
    AMLIP_CPP_DllAPI types::JobSolutionDataType request_job_solution(
            const types::JobDataType& data,
            types::AmlipIdDataType& server);

protected:

    /**
     * @brief Reference to the MultiService Client that sends jobs.
     *
     * This is created from DDS Participant in ParentNode, and its destruction is handled by ParentNode.
     */
    std::shared_ptr<dds::MultiServiceClient<types::JobDataType, types::JobSolutionDataType>> job_client_;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_MAINNODE_HPP */
