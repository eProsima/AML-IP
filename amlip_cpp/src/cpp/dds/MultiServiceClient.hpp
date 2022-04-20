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
 * @file MultiServiceClient.hpp
 */

#ifndef AMLIP__SRC_CPP_AMLIPCPP_DDS_MULTISERVICECLIENT_HPP
#define AMLIP__SRC_CPP_AMLIPCPP_DDS_MULTISERVICECLIENT_HPP

#include <dds/DdsHandler.hpp>
#include <dds/DirectWriter.hpp>
#include <dds/Reader.hpp>
#include <dds/TargetedReader.hpp>
#include <dds/Writer.hpp>
#include <types/MsRequestDataType.hpp>
#include <types/MsReferenceDataType.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

/**
 * TODO
 */
template <typename Data, typename Solution>
class MultiServiceClient
{

    // Force T to be subclass of InterfaceDataType
    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, Data);
    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, Solution);

public:

    MultiServiceClient(
        const std::string& topic,
        ddsrouter::utils::LesseePtr<DdsHandler> dds_handler);

protected:

    Writer<types::MsRequestDataType> request_availability_writer_;

    TargetedReader<types::MsReferenceDataType> reply_available_reader_;

    Writer<types::MsReferenceDataType> task_target_writer_;

    DirectWriter<Data> task_data_writer_;

    TargetedReader<Solution> task_solution_reader_;

    std::string topic_;

};

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <dds/impl/MultiServiceClient.ipp>

#endif /* AMLIP__SRC_CPP_AMLIPCPP_DDS_MULTISERVICECLIENT_HPP */
