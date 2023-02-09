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

#ifndef AMLIPCPP__SRC_CPP_NODE_INFERENCENODE_HPP
#define AMLIPCPP__SRC_CPP_NODE_INFERENCENODE_HPP

#include <functional>

#include <amlip_cpp/node/ParentNode.hpp>
#include <amlip_cpp/types/inference/InferenceDataType.hpp>
#include <amlip_cpp/types/inference/InferenceSolutionDataType.hpp>

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
 * @brief Object that is called when a new Inference data has been received.
 *
 * This class is supposed to be implemented by a User and be given to a \c InferenceNode in order to process a Inference.
 * When a Inference is received, \c process_inference is called and it is expected to return a Solution for such inference.
 */
class AMLIP_CPP_DllAPI InferenceListener
{
public:

    //! Default virtual dtor so it can be inherited.
    virtual ~InferenceListener() = default;

    /**
     * @brief Method that will be called with the Inference message received
     *
     * @param inference new Inference message received.
     *
     * @return Solution to the \c inference .
     */
    virtual types::InferenceSolutionDataType process_inference (
            const types::InferenceDataType& inference) const = 0;
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
class InferenceNode : public ParentNode
{
public:

    /**
     * @brief Construct a new Inference Node object.
     *
     * @param name name of the Node (it is advisable to be unique, or at least representative).
     */
    AMLIP_CPP_DllAPI InferenceNode(
            const char* name);

    //! Same as previous ctor but with a string argument.
    AMLIP_CPP_DllAPI InferenceNode(
            const std::string& name);

    /**
     * @brief Destroy the Inference Node object and its internal DDS entities.
     *
     * @pre Cannot be destroyed while processing an inference. Otherwise undefined behaviour.
     */
    AMLIP_CPP_DllAPI ~InferenceNode();

    /**
     * @brief Wait for data to be received and give a inference by \c callback .
     *
     * This sets the status of this Node as available to receive data, and waits for a Edge Node to ask for an inference.
     * Once the MultiService handshake has been done with a Edge Node, it will send a data to this Node and it will
     * be inferred by \c callback , that must give the \c InferenceSolutionDataType for the data.
     *
     * @attention this method is synchronous and will not finish until the inference has been solved.
     *
     * @todo asynchronous mode
     *
     * @param callback [in] function that receives a \c InferenceDataType and returns a \c InferenceSolutionDataType .
     * @param client_id [out] Id of the client that sent the Inference.
     */

    AMLIP_CPP_DllAPI void process_inference(
            const std::function<types::InferenceSolutionDataType(const types::InferenceDataType&)>& callback,
            types::AmlipIdDataType& client_id);

    //! Same as previous \c process_inference without client_id return parameter.
    AMLIP_CPP_DllAPI void process_inference(
            const std::function<types::InferenceSolutionDataType(const types::InferenceDataType&)>& callback);

    /**
     * @brief Wait for data to be received and give a inference by \c listener \c process_inference method .
     *
     * This sets the status of this Node as available to receive Inferences, and waits for a Edge Node to ask for an inference.
     * Once the MultiService handshake has been done with a Edge Node, it will send data to this Node and it will
     * be resolved by calling \c process_inference of \c listener , that must give the \c InferenceSolutionDataType for the data.
     *
     * @attention this method is synchronous and will not finish until the inference has been solved.
     *
     * @todo asynchronous mode
     *
     * @param listener [in] listener to call with a \c InferenceDataType and returns a \c InferenceSolutionDataType .
     * @param client_id [out] Id of the client that sent the Inference.
     */
    AMLIP_CPP_DllAPI void process_inference(
            const InferenceListener& listener,
            types::AmlipIdDataType& client_id);

    //! Same as previous \c process_inference without client_id return parameter.
    AMLIP_CPP_DllAPI void process_inference(
            const InferenceListener& listener);

protected:

    /**
     * @brief Reference to the MultiService Server that sends inferences.
     *
     * This is created from DDS Participant in ParentNode, and its destruction is handled by ParentNode.
     */
    std::shared_ptr<dds::MultiServiceServer<types::InferenceDataType,
            types::InferenceSolutionDataType>> inference_server_;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_INFERENCENODE_HPP */
