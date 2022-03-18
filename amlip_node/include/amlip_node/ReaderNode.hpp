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
 * @file ReaderNode.hpp
 */

#ifndef AMLIP_AMLIPNODE_READERNODE_HPP
#define AMLIP_AMLIPNODE_READERNODE_HPP

namespace eprosima {
namespace amlip {

// Forward declaration of dds
namespace dds {
class Reader;
} /* namespace dds */

namespace node {

class ReaderNode
{
public:

    ReaderNode(std::string topic_name);

    virtual ~ReaderNode();

    void start();

    void stop();

protected:

    dds::Reader* reader_;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP_AMLIPNODE_READERNODE_HPP */
