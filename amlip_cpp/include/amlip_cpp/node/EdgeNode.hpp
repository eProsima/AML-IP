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
 * @file EdgeNode.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_NODE_EDGENODE_HPP
#define AMLIPCPP__SRC_CPP_NODE_EDGENODE_HPP

#include <functional>

#include <amlip_cpp/node/ParentNode.hpp>
#include <amlip_cpp/types/inference/InferenceDataType.hpp>
#include <amlip_cpp/types/inference/InferenceSolutionDataType.hpp>

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
 * @brief This is a specialization of AML-IP Node that sends data and waits for the Inferred Solution.
 *
 * Edge Nodes are the ones in charge of sending data and collecting the inferred solution.
 * Using \c request_inferred_solution it will send the data to a Inference Node that is available, and will wait for
 * the Solution retrieved by such Node.
 *
 * @todo implement an asynchronous request_inferred_solution method.
 *
 * @warning Not Thread Safe (yet) (TODO)
 */
class EdgeNode : public ParentNode
{
public:

    /**
     * @brief Construct a new Edge Node object.
     *
     * @param name name of the Node (it is advisable to be unique, or at least representative).
     */
    AMLIP_CPP_DllAPI EdgeNode(
            const char* name);

    //! Same as previous ctor but with a string argument.
    AMLIP_CPP_DllAPI EdgeNode(
            const std::string& name);

    /**
     * @brief Destroy the Edge Node object and its internal DDS entities.
     *
     * @pre Cannot be destroyed while processing data. Otherwise undefined behaviour.
     */
    AMLIP_CPP_DllAPI ~EdgeNode();

    /**
     * @brief Send a data to any Inference Node available and wait for the solution.
     *
     * @param data [in] data to send.
     *
     * @attention this method is synchronous and will not finish until the data has been inferred.
     *
     * @todo asynchronous mode
     *
     * @return Inference data
     */
    AMLIP_CPP_DllAPI types::InferenceSolutionDataType request_inferred_solution(
            const types::InferenceDataType& data);

    /**
     * @brief Send data to any Inference Node available and wait for the solution, getting the Id of the solver.
     *
     * @param data [in] data to send.
     * @param server [out] Id of the Node that has answered.
     *
     * @attention this method is synchronous and will not finish until the data has been inferred.
     *
     * @todo asynchronous mode
     *
     * @return Inference data
     */
    AMLIP_CPP_DllAPI types::InferenceSolutionDataType request_inferred_solution(
            const types::InferenceDataType& data,
            types::AmlipIdDataType& server);

protected:

    /**
     * @brief Reference to the MultiService Client that sends inferences.
     *
     * This is created from DDS Participant in ParentNode, and its destruction is handled by ParentNode.
     */
    std::shared_ptr<dds::MultiServiceClient<types::InferenceDataType,
            types::InferenceSolutionDataType>> inference_client_;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_EDGENODE_HPP */
