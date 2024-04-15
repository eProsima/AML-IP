// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file AsyncEdgeNode.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_NODE_ASYNCEDGENODE_HPP
#define AMLIPCPP__SRC_CPP_NODE_ASYNCEDGENODE_HPP


#include <functional>

#include <amlip_cpp/types/id/TaskId.hpp>
#include <amlip_cpp/types/inference/InferenceDataType.hpp>
#include <amlip_cpp/types/inference/InferenceSolutionDataType.hpp>
#include <amlip_cpp/node/ParentNode.hpp>

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
class InferenceSolutionListener
{
public:

    //! Default virtual dtor so it can be inherited.
    virtual ~InferenceSolutionListener() = default;

    /**
     * @brief Method that will be called with each Inference received
     *
     * @param inference inference to a task previously sent.
     * @param task_id Id of the task which this inference answers.
     * @param server_id Id of the ComputingNode answering.
     */
    virtual void inference_received(
            const types::InferenceSolutionDataType& inference,
            const types::TaskId& task_id,
            const types::AmlipIdDataType& server_id) = 0;
};

/**
 * @brief This is a specialization of AML-IP Node that sends data and waits for the Inferred Solution.
 *
 * Edge Nodes are the ones in charge of sending data and collecting the inferred solution.
 * Using \c request_inferred_solution it will send the data to a Inference Node that is available, and will wait for
 * the Solution retrieved by such Node.
 *
 * @warning Not Thread Safe (yet) (TODO)
 */
class AsyncEdgeNode : public ParentNode
{
public:

    /**
     * @brief Construct a new Edge Node object.
     *
     * @param name name of the Node (it is advisable to be unique, or at least representative).
     */
    AMLIP_CPP_DllAPI AsyncEdgeNode(
            const char* name,
            const std::shared_ptr<InferenceSolutionListener>& listener,
            uint32_t domain_id);

    AMLIP_CPP_DllAPI AsyncEdgeNode(
            const char* name,
            const std::shared_ptr<InferenceSolutionListener>& listener);

    //! Same as previous ctor but with a string argument.
    AMLIP_CPP_DllAPI AsyncEdgeNode(
            const std::string& name,
            const std::shared_ptr<InferenceSolutionListener>& listener);

    /**
     * @brief Destroy the Edge Node object and its internal DDS entities.
     *
     * @pre Cannot be destroyed while processing data. Otherwise undefined behaviour.
     */
    AMLIP_CPP_DllAPI ~AsyncEdgeNode();

    /**
     * @brief Send a data to any Inference Node available and wait for the solution.
     *
     * @param data [in] data to send.
     *
     * @attention this method is synchronous and will not finish until the data has been inferred.
     *
     * @return Inference data
     */
    AMLIP_CPP_DllAPI types::TaskId request_inference(
            const types::InferenceDataType& data);

    AMLIP_CPP_DllAPI types::TaskId request_inference(
            const std::shared_ptr<types::InferenceDataType>& data);

protected:

    /**
     * @brief Reference to the MultiService Client that sends inferences.
     *
     * This is created from DDS Participant in ParentNode, and its destruction is handled by ParentNode.
     */
    std::shared_ptr<dds::AsyncMultiServiceClient<types::InferenceDataType,
            types::InferenceSolutionDataType>> inference_client_;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_ASYNCEDGENODE_HPP */
