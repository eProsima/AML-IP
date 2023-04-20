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
 * @file ComputingNode.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_NODE_COMPUTINGNODE_HPP
#define AMLIPCPP__SRC_CPP_NODE_COMPUTINGNODE_HPP

#include <functional>

#include <amlip_cpp/node/ParentNode.hpp>
#include <amlip_cpp/types/job/JobDataType.hpp>
#include <amlip_cpp/types/job/JobSolutionDataType.hpp>

// Forward declaration of dds classes
namespace eprosima {
namespace amlip {
namespace dds {

template <typename Task, typename Solution>
class MultiServiceServer;

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
     * @brief Method that will be called with the Job message received
     *
     * @param job new Job message received.
     *
     * @return Solution to the \c job .
     */
    virtual types::JobSolutionDataType process_job (
            const types::JobDataType& job) const = 0;
};

/**
 * @brief This is a specialization of AML-IP Node that waits for Job data and retrieves a Solution.
 *
 * Computing Nodes are the ones in charge of receiving training data from a Main Node
 * Using \c process_job will wait for a Main Node to send it the data for a Job, and will process this Job by
 * the Listener or callback given, and return the Solution calculated.
 *
 * @todo implement an asynchronous request_job_solution method.
 *
 * @warning Not Thread Safe (yet) (TODO)
 */
class ComputingNode : public ParentNode
{
public:

    AMLIP_CPP_DllAPI ComputingNode(
            const char* name,
            uint32_t domain_id);

    /**
     * @brief Construct a new Computing Node object.
     *
     * @param name name of the Node (it is advisable to be unique, or at least representative).
     */
    AMLIP_CPP_DllAPI ComputingNode(
            const char* name);

    //! Same as previous ctor but with a string argument.
    AMLIP_CPP_DllAPI ComputingNode(
            const std::string& name);

    /**
     * @brief Destroy the Main Node object and its internal DDS entities.
     *
     * @pre Cannot be destroyed while processing a job. Otherwise undefined behaviour.
     */
    AMLIP_CPP_DllAPI ~ComputingNode();

    /**
     * @brief Wait for a Job to be received and give a solution by \c callback .
     *
     * This sets the status of this Node as available to receive Jobs, and waits for a Main Node to ask for a Computing.
     * Once the MultiService handshake has been done with a Main Node, it will send a Job data to this Node and it will
     * be resolved by \c callback , that must give the \c JobSolutionDataType for the job.
     *
     * @attention this method is synchronous and will not finish until the job has been solved.
     *
     * @todo asynchronous mode
     *
     * @param callback [in] function that receives a \c JobDataType and returns a \c JobSolutionDataType .
     * @param client_id [out] Id of the client that sent the Job.
     */

    AMLIP_CPP_DllAPI void process_job(
            const std::function<types::JobSolutionDataType(const types::JobDataType&)>& callback,
            types::AmlipIdDataType& client_id);

    //! Same as previous \c process_job without client_id return parameter.
    AMLIP_CPP_DllAPI void process_job(
            const std::function<types::JobSolutionDataType(const types::JobDataType&)>& callback);

    /**
     * @brief Wait for a Job to be received and give a solution by \c listener \c process_job method .
     *
     * This sets the status of this Node as available to receive Jobs, and waits for a Main Node to ask for a Computing.
     * Once the MultiService handshake has been done with a Main Node, it will send a Job data to this Node and it will
     * be resolved by calling \c process_job of \c listener , that must give the \c JobSolutionDataType for the job.
     *
     * @attention this method is synchronous and will not finish until the job has been solved.
     *
     * @todo asynchronous mode
     *
     * @param listener [in] listener to call with a \c JobDataType and returns a \c JobSolutionDataType .
     * @param client_id [out] Id of the client that sent the Job.
     */
    AMLIP_CPP_DllAPI void process_job(
            const JobListener& listener,
            types::AmlipIdDataType& client_id);

    //! Same as previous \c process_job without client_id return parameter.
    AMLIP_CPP_DllAPI void process_job(
            const JobListener& listener);

protected:

    /**
     * @brief Reference to the MultiService Server that sends jobs.
     *
     * This is created from DDS Participant in ParentNode, and its destruction is handled by ParentNode.
     */
    std::shared_ptr<dds::MultiServiceServer<types::JobDataType, types::JobSolutionDataType>> job_server_;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_COMPUTINGNODE_HPP */
