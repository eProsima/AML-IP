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
 * @file JobSolutionDataType.cpp
 */

#include <iostream>

#include <amlip_cpp/types/job/JobSolutionDataType.hpp>

namespace eprosima {
namespace amlip {
namespace types {

JobSolutionDataType::JobSolutionDataType()
    : GenericDataType()
{
    // Do nothing
}

JobSolutionDataType::JobSolutionDataType(
        const std::vector<ByteType>& bytes)
    : GenericDataType(bytes)
{
    // Do nothing
}

JobSolutionDataType::JobSolutionDataType(
        const std::string& bytes)
    : GenericDataType(bytes)
{
    std::cout << "Creating solution data type: " << bytes << std::endl;
    // Do nothing
}

JobSolutionDataType::~JobSolutionDataType()
{
}

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */
