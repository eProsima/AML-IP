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
 * @file StatusNode.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_NODE_STATUSNODE_HPP
#define AMLIPCPP__SRC_CPP_NODE_STATUSNODE_HPP

#include <functional>
#include <thread>

#include <ddsrouter_utils/memory/OwnerPtr.hpp>
#include <ddsrouter_utils/ReturnCode.hpp>

#include <types/status/StatusDataType.hpp>
#include <node/ParentNode.hpp>

namespace eprosima {
namespace amlip {
namespace node {

/**
 * @brief TODO
 *
 * @warning Not Thread Safe (yet) (TODO)
 */
class StatusNode : public ParentNode
{
public:

    StatusNode(const char* name);
    StatusNode(const std::string& name);

    ~StatusNode();

    /**
     * Execute in a new thread a passive listening in Status topic and execute the callback given
     *
     * @pre There could only be called once per instance before calling \c stop_processing
     */
    void process_status_async(
            const std::function<void(const types::StatusDataType&)>& callback);

    /**
     * Stop the internal thread that is running created in \c process_status_async
     */
    void stop_processing();

protected:

    void process_routine_(
            const std::function<void(const types::StatusDataType&)>& callback);

    std::atomic<bool> processing_;

    std::thread process_thread_;

    std::shared_ptr<dds::Reader<types::StatusDataType>> status_reader_;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_NODE_STATUSNODE_HPP */
