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
 * @file StatusAmlipNodeFunctor.hpp
 */

#ifndef AMLIP_AMLIPNODE_STATUSAMLIPNODEFUNCTOR_HPP
#define AMLIP_AMLIPNODE_STATUSAMLIPNODEFUNCTOR_HPP

#include <iostream>

#include <amlip_node/types/Status.hpp>

namespace eprosima {
namespace amlip {
namespace node {

class StatusAmlipNodeFunctor
{
public:
    StatusAmlipNodeFunctor() {};
    virtual ~StatusAmlipNodeFunctor() {};
    // virtual bool operator() (types::Status status) const {std::cout << "NO CALLBACK SET OPERATOR()" << std::endl; return true;};
    virtual bool operator() (types::Status status) const = 0;
};

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP_AMLIPNODE_STATUSAMLIPNODEFUNCTOR_HPP */
