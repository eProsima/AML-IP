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
#include <amlip_cpp/types/job/SolutionDataType.hpp>

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
 * @brief TODO
 *
 * @warning Not Thread Safe (yet) (TODO)
 */
class MainNode : public ParentNode
{
public:

    MainNode(
            const char* name);
    MainNode(
            const std::string& name);

    ~MainNode();

    types::SolutionDataType request_job_solution(
            const types::JobDataType& data);

protected:

    std::shared_ptr<dds::MultiServiceClient<types::JobDataType, types::SolutionDataType>> job_client_;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_MAINNODE_HPP */
