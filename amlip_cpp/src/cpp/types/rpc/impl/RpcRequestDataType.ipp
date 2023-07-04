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
 * @file RpcRequestDataType.cpp
 */

#ifndef AMLIPCPP__SRC_CPP_TYPES_IMPL_RPCREQUESTDATATYPE_IPP
#define AMLIPCPP__SRC_CPP_TYPES_IMPL_RPCREQUESTDATATYPE_IPP

namespace eprosima {
namespace amlip {
namespace types {

template <typename T>
const char* RpcRequestDataType<T>::DATA_TYPE_PREFIX_NAME_ = "rpc_request";

template <typename T>
RpcRequestDataType<T>::RpcRequestDataType()
{
}

template <typename T>
RpcRequestDataType<T>::RpcRequestDataType(
        const AmlipIdDataType client_id,
        const TaskId& task_id,
        const T& data)
    : client_id_(client_id)
    , task_id_(task_id)
    , data_(data)
{
}

template <typename T>
RpcRequestDataType<T>::~RpcRequestDataType()
{
}

template <typename T>
RpcRequestDataType<T>::RpcRequestDataType(
        const RpcRequestDataType& x)
{
    client_id_ = x.client_id_;
    task_id_ = x.task_id_;
}

template <typename T>
RpcRequestDataType<T>::RpcRequestDataType(
        RpcRequestDataType&& x)
{
    client_id_ = std::move(x.client_id_);
    task_id_ = std::move(x.task_id_);
}

template <typename T>
RpcRequestDataType<T>& RpcRequestDataType<T>::operator =(
        const RpcRequestDataType<T>& x)
{
    client_id_ = x.client_id_;
    task_id_ = x.task_id_;

    return *this;
}

template <typename T>
RpcRequestDataType<T>& RpcRequestDataType<T>::operator =(
        RpcRequestDataType<T>&& x)
{
    client_id_ = std::move(x.client_id_);
    task_id_ = std::move(x.task_id_);

    return *this;
}

template <typename T>
bool RpcRequestDataType<T>::operator ==(
        const RpcRequestDataType<T>& x) const
{
    return (client_id_ == x.client_id_ && task_id_ == x.task_id_);
}

template <typename T>
bool RpcRequestDataType<T>::operator !=(
        const RpcRequestDataType<T>& x) const
{
    return !(*this == x);
}

template <typename T>
bool RpcRequestDataType<T>::operator <(
        const RpcRequestDataType<T>& x) const
{
    if (client_id_ < x.client_id_)
    {
        return true;
    }
    else if (x.client_id_ < client_id_)
    {
        return false;
    }
    else
    {
        return (task_id_ < x.task_id_);
    }
}

template <typename T>
AmlipIdDataType RpcRequestDataType<T>::client_id() const
{
    return client_id_;
}

template <typename T>
void RpcRequestDataType<T>::client_id(
        const AmlipIdDataType& new_value)
{
    client_id_ = new_value;
}

template <typename T>
TaskId RpcRequestDataType<T>::task_id() const
{
    return task_id_;
}

template <typename T>
void RpcRequestDataType<T>::task_id(
        const TaskId& new_value)
{
    task_id_ = new_value;
}

template <typename T>
void RpcRequestDataType<T>::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{
    scdr << client_id_;
    scdr << task_id_;
    scdr << data_;
}

template <typename T>
void RpcRequestDataType<T>::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{
    dcdr >> client_id_;
    dcdr >> task_id_;
    dcdr >> data_;
}

template <typename T>
void RpcRequestDataType<T>::serialize_key(
        eprosima::fastcdr::Cdr&) const
{
}

template <typename T>
size_t RpcRequestDataType<T>::get_max_cdr_serialized_size(
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;

    current_alignment += AmlipIdDataType::get_max_cdr_serialized_size(current_alignment);
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += T::get_max_cdr_serialized_size(current_alignment);

    return current_alignment - initial_alignment;
}

template <typename T>
size_t RpcRequestDataType<T>::get_cdr_serialized_size(
        const RpcRequestDataType& data,
        size_t current_alignment)
{
    size_t initial_alignment = current_alignment;

    current_alignment += AmlipIdDataType::get_max_cdr_serialized_size(current_alignment);
    current_alignment += 4 + eprosima::fastcdr::Cdr::alignment(current_alignment, 4);

    current_alignment += T::get_cdr_serialized_size(data.data(), current_alignment);

    return current_alignment - initial_alignment;
}

template <typename T>
size_t RpcRequestDataType<T>::get_key_max_cdr_serialized_size(
        size_t current_alignment)
{
    return current_alignment;
}

template <typename T>
bool RpcRequestDataType<T>::is_key_defined()
{
    return false;
}

template <typename T>
bool RpcRequestDataType<T>::is_bounded()
{
    return T::is_bounded();
}

template <typename T>
bool RpcRequestDataType<T>::is_plain()
{
    return T::is_plain();
}

template <typename T>
bool RpcRequestDataType<T>::construct_sample(
        void* memory)
{
    if (!is_plain())
    {
        return false;
    }
    else
    {
        new (memory) RpcRequestDataType<T>();
        return true;
    }
}

template <typename T>
std::string RpcRequestDataType<T>::type_name()
{
    // NOTE: there is no easy way to concatenate 2 const chars
    std::string result(DATA_TYPE_PREFIX_NAME_);
    result.append(T::type_name());

    return result;
}

template <typename T>
const T& RpcRequestDataType<T>::data() const
{
    return data_;
}

template <typename T>
void RpcRequestDataType<T>::data(
        T new_value)
{
    data_ = new_value;
}

template <typename T>
std::ostream& operator <<(
        std::ostream& os,
        const RpcRequestDataType<T>& request)
{
    os << "RPC-REQUEST{" << request.client_id() << "|" << request.task_id() << "}";
    return os;
}

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

#endif // AMLIPCPP__SRC_CPP_TYPES_IMPL_RPCREQUESTDATATYPE_IPP
