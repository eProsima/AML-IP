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
 * @file ComputationalNode.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_NODE_COMPUTINGNODE_HPP
#define AMLIPCPP__SRC_CPP_NODE_COMPUTINGNODE_HPP

#include <functional>

#include <node/ParentNode.hpp>
#include <types/job/JobDataType.hpp>
#include <types/job/SolutionDataType.hpp>

namespace eprosima {
namespace amlip {
namespace node {

/**
 * @brief TODO
 *
 * @warning Not Thread Safe (yet) (TODO)
 */
class ComputationalNode : public ParentNode
{
public:

    ComputationalNode(const char* name);
    ComputationalNode(const std::string& name);

    ~ComputationalNode();

    types::MsReferenceDataType process_job(
            const std::function<types::SolutionDataType(const types::JobDataType&)>& callback);

protected:

    std::shared_ptr<dds::MultiServiceServer<types::JobDataType, types::SolutionDataType>> job_server_;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_COMPUTINGNODE_HPP */
