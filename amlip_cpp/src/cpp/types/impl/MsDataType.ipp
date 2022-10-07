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
 * @file MsDataType.ipp
 */

#ifndef AMLIPCPP__SRC_CPP_TYPES_IMPL_MSDATATYPE_IPP
#define AMLIPCPP__SRC_CPP_TYPES_IMPL_MSDATATYPE_IPP

namespace eprosima {
namespace amlip {
namespace types {

template <typename T>
const char* MsDataType<T>::DATA_TYPE_PREFIX_NAME_ = "ms_data_";

template <typename T>
MsDataType<T>::MsDataType()
{
}

template <typename T>
MsDataType<T>::MsDataType(
        const AmlipIdDataType& client_id,
        const TaskId& task_id,
        const AmlipIdDataType& server_id,
        const T& data)
    : MsReferenceDataType(client_id, task_id, server_id)
    , data_(data)
{
}

template <typename T>
MsDataType<T>::MsDataType(
        const MsReferenceDataType& reference,
        const T& data)
    : MsReferenceDataType(reference)
    , data_(data)
{
}

template <typename T>
void MsDataType<T>::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{
    MsReferenceDataType::serialize(scdr);
    scdr << data_;
}

template <typename T>
void MsDataType<T>::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{
    MsReferenceDataType::deserialize(dcdr);
    dcdr >> data_;
}

template <typename T>
void MsDataType<T>::serialize_key(
        eprosima::fastcdr::Cdr& cdr) const
{
}

template <typename T>
size_t MsDataType<T>::get_max_cdr_serialized_size(
            size_t current_alignment /* = 0 */)
{
    size_t initial_alignment = current_alignment;

    current_alignment += MsReferenceDataType::get_max_cdr_serialized_size(current_alignment);

    current_alignment += T::get_max_cdr_serialized_size(current_alignment);

    return current_alignment - initial_alignment;
}

template <typename T>
size_t MsDataType<T>::get_cdr_serialized_size(
            const MsDataType& data,
            size_t current_alignment /* = 0 */)
{
    size_t initial_alignment = current_alignment;

    current_alignment += MsReferenceDataType::get_cdr_serialized_size(data, current_alignment);

    current_alignment += T::get_cdr_serialized_size(data.data(), current_alignment);

    return current_alignment - initial_alignment;
}

template <typename T>
size_t MsDataType<T>::get_key_max_cdr_serialized_size(
            size_t current_alignment /* = 0 */)
{
    return current_alignment;
}

template <typename T>
bool MsDataType<T>::is_key_defined()
{
    return false;
}

template <typename T>
bool MsDataType<T>::is_bounded()
{
    return T::is_bounded();
}

template <typename T>
bool MsDataType<T>::is_plain()
{
    return T::is_plain();
}

template <typename T>
bool MsDataType<T>::construct_sample(
            void* memory)
{
    if (!is_plain())
    {
        return false;
    }
    else
    {
        new (memory) MsDataType<T>();
        return true;
    }
}

template <typename T>
std::string MsDataType<T>::type_name()
{
    // NOTE: there is no easy way to concatenate 2 const chars
    std::string result(DATA_TYPE_PREFIX_NAME_);
    result.append(T::type_name());

    return result;
}

template <typename T>
const T& MsDataType<T>::data() const
{
    return data_;
}

template <typename T>
void MsDataType<T>::data(T new_value)
{
    data_ = new_value;
}

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

#endif // AMLIPCPP__SRC_CPP_TYPES_IMPL_MSDATATYPE_IPP
