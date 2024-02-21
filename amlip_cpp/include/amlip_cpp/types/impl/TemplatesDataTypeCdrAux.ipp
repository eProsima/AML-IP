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
 * @file TemplatesDataTypeCdrAux.cpp
 */

#pragma once


namespace eprosima {
namespace fastcdr {

class Cdr;
class CdrSizeCalculator;

template <typename T>
AMLIP_CPP_DllAPI size_t calculate_serialized_size(
        eprosima::fastcdr::CdrSizeCalculator& calculator,
        const eprosima::amlip::types::MsDataType<T>& data,
        size_t& current_alignment);

template <typename T>
AMLIP_CPP_DllAPI void serialize(
        eprosima::fastcdr::Cdr& scdr,
        const eprosima::amlip::types::MsDataType<T>& data);

template <typename T>
AMLIP_CPP_DllAPI void deserialize(
        eprosima::fastcdr::Cdr& cdr,
        eprosima::amlip::types::MsDataType<T>& data);

template <typename T>
AMLIP_CPP_DllAPI size_t calculate_serialized_size(
        eprosima::fastcdr::CdrSizeCalculator& calculator,
        const eprosima::amlip::types::RpcRequestDataType<T>& data,
        size_t& current_alignment);

template <typename T>
AMLIP_CPP_DllAPI void  serialize(
        eprosima::fastcdr::Cdr& scdr,
        const eprosima::amlip::types::RpcRequestDataType<T>& data);

template <typename T>
AMLIP_CPP_DllAPI void deserialize(
        eprosima::fastcdr::Cdr& cdr,
        eprosima::amlip::types::RpcRequestDataType<T>& data);

template <typename T>
AMLIP_CPP_DllAPI size_t calculate_serialized_size(
        eprosima::fastcdr::CdrSizeCalculator& calculator,
        const eprosima::amlip::types::RpcReplyDataType<T>& data,
        size_t& current_alignment);

template <typename T>
AMLIP_CPP_DllAPI void  serialize(
        eprosima::fastcdr::Cdr& scdr,
        const eprosima::amlip::types::RpcReplyDataType<T>& data);

template <typename T>
AMLIP_CPP_DllAPI void deserialize(
        eprosima::fastcdr::Cdr& cdr,
        eprosima::amlip::types::RpcReplyDataType<T>& data);

} // namespace fastcdr
} // namespace eprosima

#include <fastcdr/Cdr.h>
#include <fastcdr/CdrSizeCalculator.hpp>


#include <fastcdr/exceptions/BadParamException.h>
using namespace eprosima::fastcdr::exception;

namespace eprosima {
namespace fastcdr {

template <typename T>
AMLIP_CPP_DllAPI size_t calculate_serialized_size(
        eprosima::fastcdr::CdrSizeCalculator& calculator,
        const amlip::types::MsDataType<T>& data,
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
                    data.client_id(), current_alignment);

    calculated_size += calculator.calculate_member_serialized_size(eprosima::fastcdr::MemberId(1),
                    data.task_id(), current_alignment);

    calculated_size += calculator.calculate_member_serialized_size(eprosima::fastcdr::MemberId(2),
                    data.server_id(), current_alignment);

    calculated_size += calculator.calculate_member_serialized_size(eprosima::fastcdr::MemberId(3),
                    data.data(), current_alignment);

    calculated_size += calculator.end_calculate_type_serialized_size(previous_encoding, current_alignment);

    return calculated_size;
}

template <typename T>
AMLIP_CPP_DllAPI void serialize(
        eprosima::fastcdr::Cdr& scdr,
        const amlip::types::MsDataType<T>& data)
{
    eprosima::fastcdr::Cdr::state current_state(scdr);
    scdr.begin_serialize_type(current_state,
            eprosima::fastcdr::CdrVersion::XCDRv2 == scdr.get_cdr_version() ?
            eprosima::fastcdr::EncodingAlgorithmFlag::DELIMIT_CDR2 :
            eprosima::fastcdr::EncodingAlgorithmFlag::PLAIN_CDR);

    scdr
        << eprosima::fastcdr::MemberId(0) << data.client_id()
        << eprosima::fastcdr::MemberId(1) << data.task_id()
        << eprosima::fastcdr::MemberId(2) << data.server_id()
        << eprosima::fastcdr::MemberId(3) << data.data()
    ;
    scdr.end_serialize_type(current_state);
}

template <typename T>
AMLIP_CPP_DllAPI void deserialize(
        eprosima::fastcdr::Cdr& cdr,
        amlip::types::MsDataType<T>& data)
{
    cdr.deserialize_type(eprosima::fastcdr::CdrVersion::XCDRv2 == cdr.get_cdr_version() ?
            eprosima::fastcdr::EncodingAlgorithmFlag::DELIMIT_CDR2 :
            eprosima::fastcdr::EncodingAlgorithmFlag::PLAIN_CDR,
            [&data](eprosima::fastcdr::Cdr& dcdr, const eprosima::fastcdr::MemberId& mid) -> bool
            {
                bool ret_value = true;
                switch (mid.id)
                {
                    case 0:
                        dcdr >> data.client_id();
                        break;

                    case 1:
                        dcdr >> data.task_id();
                        break;

                    case 2:
                        dcdr >> data.server_id();
                        break;

                    case 3:
                        dcdr >> data.data();
                        break;

                    default:
                        ret_value = false;
                        break;
                }
                return ret_value;
            });
}

template <typename T>
AMLIP_CPP_DllAPI size_t calculate_serialized_size(
        eprosima::fastcdr::CdrSizeCalculator& calculator,
        const amlip::types::RpcRequestDataType<T>& data,
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
                    data.client_id(), current_alignment);

    calculated_size += calculator.calculate_member_serialized_size(eprosima::fastcdr::MemberId(1),
                    data.task_id(), current_alignment);

    calculated_size += calculator.calculate_member_serialized_size(eprosima::fastcdr::MemberId(2),
                    data.server_id(), current_alignment);

    calculated_size += calculator.calculate_member_serialized_size(eprosima::fastcdr::MemberId(3),
                    data.data(), current_alignment);


    calculated_size += calculator.end_calculate_type_serialized_size(previous_encoding, current_alignment);

    return calculated_size;
}

template <typename T>
AMLIP_CPP_DllAPI void  serialize(
        eprosima::fastcdr::Cdr& scdr,
        const amlip::types::RpcRequestDataType<T>& data)
{
    eprosima::fastcdr::Cdr::state current_state(scdr);
    scdr.begin_serialize_type(current_state,
            eprosima::fastcdr::CdrVersion::XCDRv2 == scdr.get_cdr_version() ?
            eprosima::fastcdr::EncodingAlgorithmFlag::DELIMIT_CDR2 :
            eprosima::fastcdr::EncodingAlgorithmFlag::PLAIN_CDR);

    scdr
        << eprosima::fastcdr::MemberId(0) << data.client_id()
        << eprosima::fastcdr::MemberId(1) << data.task_id()
        << eprosima::fastcdr::MemberId(2) << data.server_id()
        << eprosima::fastcdr::MemberId(3) << data.data()
    ;
    scdr.end_serialize_type(current_state);
}

template <typename T>
AMLIP_CPP_DllAPI void deserialize(
        eprosima::fastcdr::Cdr& cdr,
        amlip::types::RpcRequestDataType<T>& data)
{
    cdr.deserialize_type(eprosima::fastcdr::CdrVersion::XCDRv2 == cdr.get_cdr_version() ?
            eprosima::fastcdr::EncodingAlgorithmFlag::DELIMIT_CDR2 :
            eprosima::fastcdr::EncodingAlgorithmFlag::PLAIN_CDR,
            [&data](eprosima::fastcdr::Cdr& dcdr, const eprosima::fastcdr::MemberId& mid) -> bool
            {
                bool ret_value = true;
                switch (mid.id)
                {
                    case 0:
                        dcdr >> data.client_id();
                        break;
                    case 1:
                        dcdr >> data.task_id();
                        break;
                    case 2:
                        dcdr >> data.server_id();
                        break;
                    case 3:
                        dcdr >> data.data();
                        break;

                    default:
                        ret_value = false;
                        break;
                }
                return ret_value;
            });
}

template <typename T>
AMLIP_CPP_DllAPI size_t calculate_serialized_size(
        eprosima::fastcdr::CdrSizeCalculator& calculator,
        const amlip::types::RpcReplyDataType<T>& data,
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
                    data.client_id(), current_alignment);

    calculated_size += calculator.calculate_member_serialized_size(eprosima::fastcdr::MemberId(1),
                    data.task_id(), current_alignment);

    calculated_size += calculator.calculate_member_serialized_size(eprosima::fastcdr::MemberId(2),
                    data.server_id(), current_alignment);

    calculated_size += calculator.calculate_member_serialized_size(eprosima::fastcdr::MemberId(3),
                    data.data(), current_alignment);


    calculated_size += calculator.end_calculate_type_serialized_size(previous_encoding, current_alignment);

    return calculated_size;
}

template <typename T>
AMLIP_CPP_DllAPI void serialize(
        eprosima::fastcdr::Cdr& scdr,
        const amlip::types::RpcReplyDataType<T>& data)
{
    eprosima::fastcdr::Cdr::state current_state(scdr);
    scdr.begin_serialize_type(current_state,
            eprosima::fastcdr::CdrVersion::XCDRv2 == scdr.get_cdr_version() ?
            eprosima::fastcdr::EncodingAlgorithmFlag::DELIMIT_CDR2 :
            eprosima::fastcdr::EncodingAlgorithmFlag::PLAIN_CDR);

    scdr
        << eprosima::fastcdr::MemberId(0) << data.client_id()
        << eprosima::fastcdr::MemberId(1) << data.task_id()
        << eprosima::fastcdr::MemberId(2) << data.server_id()
        << eprosima::fastcdr::MemberId(3) << data.data()
    ;
    scdr.end_serialize_type(current_state);
}

template <typename T>
AMLIP_CPP_DllAPI void deserialize(
        eprosima::fastcdr::Cdr& cdr,
        amlip::types::RpcReplyDataType<T>& data)
{
    cdr.deserialize_type(eprosima::fastcdr::CdrVersion::XCDRv2 == cdr.get_cdr_version() ?
            eprosima::fastcdr::EncodingAlgorithmFlag::DELIMIT_CDR2 :
            eprosima::fastcdr::EncodingAlgorithmFlag::PLAIN_CDR,
            [&data](eprosima::fastcdr::Cdr& dcdr, const eprosima::fastcdr::MemberId& mid) -> bool
            {
                bool ret_value = true;
                switch (mid.id)
                {
                    case 0:
                        dcdr >> data.client_id();
                        break;
                    case 1:
                        dcdr >> data.task_id();
                        break;
                    case 2:
                        dcdr >> data.server_id();
                        break;
                    case 3:
                        dcdr >> data.data();
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
