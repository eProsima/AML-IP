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
 * @file ComputingAmlipNodeImpl.hpp
 */

#ifndef AMLIP__SRC_CPP_AMLIPNODE_COMPUTINGAMLIPNODEIMPL_HPP
#define AMLIP__SRC_CPP_AMLIPNODE_COMPUTINGAMLIPNODEIMPL_HPP

#include <amlip_types/Job.hpp>
#include <amlip_types/JobSolution.hpp>

#include <amlip_node/AmlipNodeImpl.hpp>
#include <amlip_dds/MultiServiceServer.hpp>

namespace eprosima {
namespace amlip {
namespace node {

class ComputingAmlipNodeImpl : public AmlipNodeImpl
{
public:

    ComputingAmlipNodeImpl();

    virtual ~ComputingAmlipNodeImpl();

    void spin();

    void stop();

protected:

    dds::MultiServiceServer<Job, JobSolution> job_server_;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP__SRC_CPP_AMLIPNODE_COMPUTINGAMLIPNODEIMPL_HPP */
