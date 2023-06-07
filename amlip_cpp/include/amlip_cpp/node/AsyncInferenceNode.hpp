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
 * @file InferenceNode.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_NODE_ASYNCINFERENCENODE_HPP
#define AMLIPCPP__SRC_CPP_NODE_ASYNCINFERENCENODE_HPP

#include <functional>

#include <amlip_cpp/node/ParentNode.hpp>
#include <amlip_cpp/types/id/TaskId.hpp>
#include <amlip_cpp/types/inference/InferenceDataType.hpp>
#include <amlip_cpp/types/inference/InferenceSolutionDataType.hpp>

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
 * @brief Object that is called when a new Inference data has been received.
 *
 * This class is supposed to be implemented by a User and be given to a \c AsyncInferenceNode in order to process a Inference.
 * When a Inference is received, \c process_inference is called and it is expected to return a Solution for such inference.
 */
class AMLIP_CPP_DllAPI InferenceReplier
{
public:

    //! Default virtual dtor so it can be inherited.
    virtual ~InferenceReplier() = default;

    /**
     * @brief Method that will be called with the inference message received to calculate an answer.
     *
     * @param inference new inference message received.
     * @param task_id Id of the Task received.
     * @param client_id Id of the Client that sent this inference.
     *
     * @return Solution to the \c inference .
     */
    virtual types::InferenceSolutionDataType process_inference (
            const types::InferenceDataType& inference,
            const types::TaskId& task_id,
            const types::AmlipIdDataType& client_id) = 0;
};

/**
 * @brief This is a specialization of AML-IP Node that waits for data and retrieves an Inference.
 *
 * Inference Nodes are the ones in charge of receiving data from a Edge Node
 * Using \c process_inference will wait for a Edge Node to send it the data for an Inference, and will process this data by
 * the Listener or callback given, and return the Inference calculated.
 *
 * @todo implement an asynchronous request_inference method.
 *
 * @warning Not Thread Safe (yet) (TODO)
 */
class AsyncInferenceNode : public ParentNode
{
public:

    /**
     * @brief Construct a new Async Inference Node object.
     *
     * @param name name of the Node (it is advisable to be unique, or at least representative).
     */
    AMLIP_CPP_DllAPI AsyncInferenceNode(
            const char* name,
            const std::shared_ptr<InferenceReplier>& listener,
            uint32_t domain_id);

    AMLIP_CPP_DllAPI AsyncInferenceNode(
            const char* name,
            const std::shared_ptr<InferenceReplier>& listener);

    //! Same as previous ctor but with a string argument.
    AMLIP_CPP_DllAPI AsyncInferenceNode(
            const std::string& name,
            const std::shared_ptr<InferenceReplier>& listener);

    /**
     * @brief Destroy the Async Inference Node object and its internal DDS entities.
     *
     * @pre Cannot be destroyed while processing an inference. Otherwise undefined behaviour.
     */
    AMLIP_CPP_DllAPI ~AsyncInferenceNode();

    /**
     * @brief Process Inferences asynchronously.
     *
     * This uses an internal thread that execute the whole multiservice process.
     * In order to calculate Inferences that are received, it uses \c process_inference from Listener given.
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
     * @brief Reference to the AsyncMultiService Server that sends inferences.
     *
     * This is created from DDS Participant in ParentNode, and its destruction is handled by ParentNode.
     */
    std::shared_ptr<dds::AsyncMultiServiceServer<types::InferenceDataType,
            types::InferenceSolutionDataType>> inference_server_;

    std::shared_ptr<InferenceReplier> listener_;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_ASYNCINFERENCENODE_HPP */
