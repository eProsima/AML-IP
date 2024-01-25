// Copyright 2024 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file GenericDataTypeCdrAux.ipp
 * This source file contains the definition of a generic type that contains void* data.
 */

#pragma once

#include <fastcdr/Cdr.h>
#include <fastcdr/CdrSizeCalculator.hpp>

#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

namespace eprosima {
namespace fastcdr {


template<>
AMLIP_CPP_DllAPI size_t calculate_serialized_size(
        eprosima::fastcdr::CdrSizeCalculator& calculator,
        const amlip::types::GenericDataType& data,
        size_t& current_alignment)
{
    static_cast<void>(data);

    eprosima::fastcdr::EncodingAlgorithmFlag previous_encoding = calculator.get_encoding();
    size_t calculated_size {calculator.begin_calculate_type_serialized_size(
                                eprosima::fastcdr::CdrVersion::XCDRv2 == calculator.get_cdr_version() ?
                                eprosima::fastcdr::EncodingAlgorithmFlag::DELIMIT_CDR2 :
                                eprosima::fastcdr::EncodingAlgorithmFlag::PLAIN_CDR,
                                current_alignment)};


    calculated_size += calculator.calculate_member_serialized_size(eprosima::fastcdr::MemberId(0),
                    data.data_size(), current_alignment);

    calculated_size += calculator.calculate_member_serialized_size(eprosima::fastcdr::MemberId(1),
                    data.has_been_allocated(), current_alignment);

    calculated_size += calculator.calculate_array_serialized_size(static_cast<uint8_t*>(data.data()),
                    data.data_size(), current_alignment);

    calculated_size += calculator.end_calculate_type_serialized_size(previous_encoding, current_alignment);

    return calculated_size;
}

template<>
AMLIP_CPP_DllAPI void serialize(
        eprosima::fastcdr::Cdr& scdr,
        const amlip::types::GenericDataType& data)
{
    eprosima::fastcdr::Cdr::state current_state(scdr);
    scdr.begin_serialize_type(current_state,
            eprosima::fastcdr::CdrVersion::XCDRv2 == scdr.get_cdr_version() ?
            eprosima::fastcdr::EncodingAlgorithmFlag::DELIMIT_CDR2 :
            eprosima::fastcdr::EncodingAlgorithmFlag::PLAIN_CDR);

    scdr
        << eprosima::fastcdr::MemberId(0) << data.data_size()
        << eprosima::fastcdr::MemberId(1) << data.has_been_allocated()
    ;
    scdr.serialize_array(static_cast<uint8_t*>(data.data()), data.data_size());
    scdr.end_serialize_type(current_state);
}

template<>
AMLIP_CPP_DllAPI void deserialize(
        eprosima::fastcdr::Cdr& cdr,
        amlip::types::GenericDataType& data)
{
    // If data has been already allocated (it has been already deserialized), we free it
    if (data.has_been_allocated())
    {
        free(data.data());
    }

    cdr.deserialize_type(eprosima::fastcdr::CdrVersion::XCDRv2 == cdr.get_cdr_version() ?
            eprosima::fastcdr::EncodingAlgorithmFlag::DELIMIT_CDR2 :
            eprosima::fastcdr::EncodingAlgorithmFlag::PLAIN_CDR,
            [&data](eprosima::fastcdr::Cdr& dcdr, const eprosima::fastcdr::MemberId& mid) -> bool
            {
                bool ret_value = true;
                switch (mid.id)
                {
                    case 0:
                        dcdr >> data.data_size();
                        break;

                    case 1:
                        bool aux;
                        dcdr >> aux;
                        data.has_been_allocated(aux);
                        break;

                    case 2:

                        // Store enough space to deserialize the data
                        data.data(malloc(data.data_size() * sizeof(uint8_t)));
                        // Deserialize array
                        dcdr.deserialize_array(static_cast<uint8_t*>(data.data()), data.data_size());
                        break;

                    default:
                        ret_value = false;
                        break;
                }
                return ret_value;
            });
}

} /* namespace fastcdr */
} /* namespace eprosima */
