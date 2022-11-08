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

/*!
 * @file SolutionDataType.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_TYPES_SOLUTIONDATATYPE_HPP
#define AMLIPCPP__SRC_CPP_TYPES_SOLUTIONDATATYPE_HPP

#include <types/GenericDataType.hpp>

namespace eprosima {
namespace amlip {
namespace types {

/*!
 * @brief AML Solution Task Data Type
 *
 * This class may be reimplemented according with AML team designs.
 *
 * TODO: change name to JobSolutionDataType
 */
class SolutionDataType : public GenericDataType
{
public:

    //! Use parent constructors
    using GenericDataType::GenericDataType;

};

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

#endif // AMLIPCPP__SRC_CPP_TYPES_SOLUTIONDATATYPE_HPP
