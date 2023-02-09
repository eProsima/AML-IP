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

/*!
 * @file InferenceSolutionDataType.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_TYPES_INFERENCESOLUTIONDATATYPE_HPP
#define AMLIPCPP__SRC_CPP_TYPES_INFERENCESOLUTIONDATATYPE_HPP

#include <string>
#include <vector>

#include <amlip_cpp/types/GenericDataType.hpp>

namespace eprosima {
namespace amlip {
namespace types {

/*!
 * @brief AML Inferred Solution Task Data Type
 *
 * This class implements the Solution DataType, the information of already inferred data from a Inference.
 * So far this class is a generic void* and size to a bunch of bytes.
 *
 * @note This class may be reimplemented according with AML team designs.
 */
class InferenceSolutionDataType : public GenericDataType
{
public:

    // NOTE: This methods are included already in GenericDataType,
    // but it is required for Windows and SWIG to work to have them declare here.
    // All constructors and specially destructor must be declared here.
    // TODO: Check if there is some way to avoid this.
    AMLIP_CPP_DllAPI InferenceSolutionDataType() = default;

    AMLIP_CPP_DllAPI InferenceSolutionDataType(
            void* data,
            const uint32_t size,
            bool take_ownership = false);

    AMLIP_CPP_DllAPI InferenceSolutionDataType(
            const std::vector<ByteType>& bytes);

    AMLIP_CPP_DllAPI InferenceSolutionDataType(
            const std::string& bytes);

    AMLIP_CPP_DllAPI virtual ~InferenceSolutionDataType() = default;

};

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

#endif // AMLIPCPP__SRC_CPP_TYPES_INFERENCESOLUTIONDATATYPE_HPP
