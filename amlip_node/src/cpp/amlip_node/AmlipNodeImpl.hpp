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
 * @file AmlipNodeImpl.hpp
 */

#ifndef AMLIP__SRC_CPP_AMLIPNODE_AMLIPNODEIMPL_HPP
#define AMLIP__SRC_CPP_AMLIPNODE_AMLIPNODEIMPL_HPP

#include <memory>

#include <amlip_dds/Participant.hpp>
#include <amlip_dds/Writer.hpp>
#include <amlip_types/AmlipId.hpp>
#include <amlip_types/Status.hpp>

namespace eprosima {
namespace amlip {
namespace node {

class AmlipNodeImpl
{
public:

    AmlipNodeImpl();

    virtual ~AmlipNodeImpl();

    types::AmlipId id() const noexcept;

protected:

    void write_status_(types::StatusKind kind);

    virtual types::NodeKind node_kind_() const noexcept;

    std::shared_ptr<dds::Participant> participant_;

    std::shared_ptr<dds::Writer<types::Status>> status_writer_;

    types::AmlipId id_;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP__SRC_CPP_AMLIPNODE_AMLIPNODEIMPL_HPP */
