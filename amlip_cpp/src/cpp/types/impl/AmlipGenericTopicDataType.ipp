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
 * @file AmlipGenericTopicDataType.ipp
 *
 * This file has been mainly generated automatically by FastDDSGen, and manually modified to fit the needs of the
 * generic type.
 *
 * TODO:
 * Change ugly names of variables
 */

#ifndef AMLIPCPP__SRC_CPP_TYPES_IMPL_AMLIPGENERICTOPICDATATYPE_IPP
#define AMLIPCPP__SRC_CPP_TYPES_IMPL_AMLIPGENERICTOPICDATATYPE_IPP

#include <fastdds/rtps/common/CdrSerialization.hpp>

#include <types/AmlipGenericTopicDataType.hpp>

namespace eprosima {
namespace amlip {
namespace types {

using SerializedPayload_t = eprosima::fastrtps::rtps::SerializedPayload_t;
using InstanceHandle_t = eprosima::fastrtps::rtps::InstanceHandle_t;
using DataRepresentationId_t = eprosima::fastdds::dds::DataRepresentationId_t;

template <class T>
AmlipGenericTopicDataType<T>::AmlipGenericTopicDataType()
{
    setName(T::type_name().c_str());
    auto type_size = T::get_max_cdr_serialized_size();
    type_size += eprosima::fastcdr::Cdr::alignment(type_size, 4); /* possible submessage alignment */
    m_typeSize = static_cast<uint32_t>(type_size) + 4; /*encapsulation*/
    m_isGetKeyDefined = T::is_key_defined();
    size_t keyLength = T::get_key_max_cdr_serialized_size() > 16 ?
            T::get_key_max_cdr_serialized_size() : 16;
    key_buffer_ = reinterpret_cast<unsigned char*>(malloc(keyLength));
    memset(key_buffer_, 0, keyLength);
}

template <class T>
AmlipGenericTopicDataType<T>::~AmlipGenericTopicDataType()
{
    if (key_buffer_ != nullptr)
    {
        free(key_buffer_);
    }
}

template <class T>
bool AmlipGenericTopicDataType<T>::serialize(
        void* data,
        SerializedPayload_t* payload,
        DataRepresentationId_t data_representation)
{
    T* p_type = static_cast<T*>(data);

    // Object that manages the raw buffer.
    eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(payload->data), payload->max_size);
    // Object that serializes the data.
    eprosima::fastcdr::Cdr ser(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
            data_representation == DataRepresentationId_t::XCDR_DATA_REPRESENTATION ?
            eprosima::fastcdr::CdrVersion::XCDRv1 : eprosima::fastcdr::CdrVersion::XCDRv2);
    payload->encapsulation = ser.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
    ser.set_encoding_flag(
        data_representation == DataRepresentationId_t::XCDR_DATA_REPRESENTATION ?
        eprosima::fastcdr::EncodingAlgorithmFlag::PLAIN_CDR  :
        eprosima::fastcdr::EncodingAlgorithmFlag::DELIMIT_CDR2);

    try
    {
        // Serialize encapsulation
        ser.serialize_encapsulation();
        // Serialize the object.
        ser << *p_type;
    }
    catch (eprosima::fastcdr::exception::Exception& /*exception*/)
    {
        return false;
    }

    // Get the serialized length
    payload->length = static_cast<uint32_t>(ser.get_serialized_data_length());
    return true;
}

template <class T>
bool AmlipGenericTopicDataType<T>::deserialize(
        SerializedPayload_t* payload,
        void* data)
{
    //Convert DATA to pointer of your type
    T* p_type = static_cast<T*>(data);

    // Object that manages the raw buffer.
    eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(payload->data), payload->length);

    // Object that deserializes the data.
    eprosima::fastcdr::Cdr deser(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN);

    // Deserialize encapsulation.
    deser.read_encapsulation();
    payload->encapsulation = deser.endianness() == eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;

    try
    {
        // Deserialize the object.
        deser >> *p_type;
    }
    catch (eprosima::fastcdr::exception::Exception& /*exception*/)
    {
        return false;
    }

    return true;
}

template <class T>
std::function<uint32_t()> AmlipGenericTopicDataType<T>::getSerializedSizeProvider(
        void* data,
        DataRepresentationId_t data_representation)
{
    return [data, data_representation]() -> uint32_t
           {
               try
               {
                   eprosima::fastcdr::CdrSizeCalculator calculator(
                       data_representation == DataRepresentationId_t::XCDR_DATA_REPRESENTATION ?
                       eprosima::fastcdr::CdrVersion::XCDRv1 :eprosima::fastcdr::CdrVersion::XCDRv2);
                   size_t current_alignment {0};
                   return static_cast<uint32_t>(calculator.calculate_serialized_size(
                              *static_cast<T*>(data), current_alignment)) +
                          4u /*encapsulation*/;
               }
               catch (eprosima::fastcdr::exception::Exception& /*exception*/)
               {
                   return 0;
               }
           };
}

template <class T>
bool AmlipGenericTopicDataType<T>::getKey(
        void* data,
        InstanceHandle_t* handle,
        bool force_md5)
{
    if (!m_isGetKeyDefined)
    {
        return false;
    }

    T* p_type = static_cast<T*>(data);

    // Object that manages the raw buffer.
    eprosima::fastcdr::FastBuffer fastbuffer(reinterpret_cast<char*>(key_buffer_),
            T::get_key_max_cdr_serialized_size());

    // Object that serializes the data.
    eprosima::fastcdr::Cdr ser(fastbuffer, eprosima::fastcdr::Cdr::BIG_ENDIANNESS);
    static_cast<void>(ser);
    static_cast<void>(*p_type);
    if (force_md5 || T::get_key_max_cdr_serialized_size() > 16)
    {
        md5_.init();
        md5_.update(key_buffer_, static_cast<unsigned int>(ser.get_serialized_data_length()));
        md5_.finalize();
        for (uint8_t i = 0; i < 16; ++i)
        {
            handle->value[i] = md5_.digest[i];
        }
    }
    else
    {
        for (uint8_t i = 0; i < 16; ++i)
        {
            handle->value[i] = key_buffer_[i];
        }
    }
    return true;
}

template <class T>
void* AmlipGenericTopicDataType<T>::createData()
{
    return reinterpret_cast<void*>(new T());
}

template <class T>
void AmlipGenericTopicDataType<T>::deleteData(
        void* data)
{
    delete(reinterpret_cast<T*>(data));
}

template <class T>
bool AmlipGenericTopicDataType<T>::is_bounded() const
{
    return T::is_bounded();
}

template <class T>
bool AmlipGenericTopicDataType<T>::is_plain() const
{
    return T::is_plain();
}

template <class T>
bool AmlipGenericTopicDataType<T>::construct_sample(
        void* memory) const
{
    if (is_plain())
    {
        // Create a new type in the already allocated memory
        new (memory) T();
        return true;
    }
    else
    {
        // If type is not plain it could not be constructed
        static_cast<void>(memory);
        return false;
    }
}

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

#endif // AMLIPCPP__SRC_CPP_TYPES_IMPL_AMLIPGENERICTOPICDATATYPE_IPP
