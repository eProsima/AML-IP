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
 * @file RpcReplyDataType.ipp
 */

#ifndef AMLIPCPP__SRC_CPP_TYPES_IMPL_RPCREPLYDATATYPE_IPP
#define AMLIPCPP__SRC_CPP_TYPES_IMPL_RPCREPLYDATATYPE_IPP

namespace eprosima {
namespace amlip {
namespace types {

template <typename T>
const char* RpcReplyDataType<T>::DATA_TYPE_PREFIX_NAME_ = "rpc_reply";

template <typename T>
RpcReplyDataType<T>::RpcReplyDataType()
{
}

template <typename T>
RpcReplyDataType<T>::RpcReplyDataType(
        const AmlipIdDataType& client_id,
        const TaskId& task_id,
        const AmlipIdDataType& server_id,
        const T& data)
    : server_id_(server_id)
    , task_id_(task_id)
    , data_(data)
{
}

template <typename T>
RpcReplyDataType<T>::~RpcReplyDataType()
{
}

template <typename T>
RpcReplyDataType<T>::RpcReplyDataType(
        const RpcReplyDataType& x)
{
    server_id_ = x.server_id_;
    task_id_ = x.task_id_;
}

template <typename T>
RpcReplyDataType<T>::RpcReplyDataType(
        RpcReplyDataType&& x)
{
    server_id_ = std::move(x.server_id_);
    task_id_ = std::move(x.task_id_);
}

template <typename T>
RpcReplyDataType<T>& RpcReplyDataType<T>::operator =(
        const RpcReplyDataType<T>& x)
{
    server_id_ = x.server_id_;
    task_id_ = x.task_id_;

    return *this;
}

template <typename T>
RpcReplyDataType<T>& RpcReplyDataType<T>::operator =(
        RpcReplyDataType<T>&& x)
{
    server_id_ = std::move(x.server_id_);
    task_id_ = std::move(x.task_id_);

    return *this;
}

template <typename T>
bool RpcReplyDataType<T>::operator ==(
        const RpcReplyDataType<T>& x) const
{
    return (server_id_ == x.server_id_ && task_id_ == x.task_id_);
}

template <typename T>
bool RpcReplyDataType<T>::operator !=(
        const RpcReplyDataType<T>& x) const
{
    return !(*this == x);
}

template <typename T>
bool RpcReplyDataType<T>::operator <(
        const RpcReplyDataType<T>& x) const
{
    if (server_id_ < x.server_id_)
    {
        return true;
    }
    else if (x.server_id_ < server_id_)
    {
        return false;
    }
    else
    {
        return (task_id_ < x.task_id_);
    }
}

template <typename T>
AmlipIdDataType RpcReplyDataType<T>::server_id() const
{
    return server_id_;
}

template <typename T>
void RpcReplyDataType<T>::server_id(
        const AmlipIdDataType& new_value)
{
    server_id_ = new_value;
}

template <typename T>
TaskId RpcReplyDataType<T>::task_id() const
{
    return task_id_;
}

template <typename T>
void RpcReplyDataType<T>::task_id(
        const TaskId& new_value)
{
    task_id_ = new_value;
}

template <typename T>
void RpcReplyDataType<T>::serialize(
        eprosima::fastcdr::Cdr& scdr) const
{
    scdr << server_id_;
    scdr << task_id_;
    scdr << data_;
}

template <typename T>
void RpcReplyDataType<T>::deserialize(
        eprosima::fastcdr::Cdr& dcdr)
{
    dcdr >> server_id_;
    dcdr >> task_id_;
    dcdr >> data_;
}

template <typename T>
void RpcReplyDataType<T>::serialize_key(
        eprosima::fastcdr::Cdr& cdr) const
{
}

template <typename T>
size_t RpcReplyDataType<T>::get_max_cdr_serialized_size(
        size_t current_alignment /* = 0 */)
{
    size_t initial_alignment = current_alignment;

    current_alignment += RpcReplyDataType::get_max_cdr_serialized_size(current_alignment);

    current_alignment += T::get_max_cdr_serialized_size(current_alignment);

    return current_alignment - initial_alignment;
}

template <typename T>
size_t RpcReplyDataType<T>::get_cdr_serialized_size(
        const RpcReplyDataType& data,
        size_t current_alignment /* = 0 */)
{
    size_t initial_alignment = current_alignment;

    current_alignment += RpcReplyDataType::get_cdr_serialized_size(data, current_alignment);

    current_alignment += T::get_cdr_serialized_size(data.data(), current_alignment);

    return current_alignment - initial_alignment;
}

template <typename T>
size_t RpcReplyDataType<T>::get_key_max_cdr_serialized_size(
        size_t current_alignment /* = 0 */)
{
    return current_alignment;
}

template <typename T>
bool RpcReplyDataType<T>::is_key_defined()
{
    return false;
}

template <typename T>
bool RpcReplyDataType<T>::is_bounded()
{
    return T::is_bounded();
}

template <typename T>
bool RpcReplyDataType<T>::is_plain()
{
    return T::is_plain();
}

template <typename T>
bool RpcReplyDataType<T>::construct_sample(
        void* memory)
{
    if (!is_plain())
    {
        return false;
    }
    else
    {
        new (memory) RpcReplyDataType<T>();
        return true;
    }
}

template <typename T>
std::string RpcReplyDataType<T>::type_name()
{
    // NOTE: there is no easy way to concatenate 2 const chars
    std::string result(DATA_TYPE_PREFIX_NAME_);
    result.append(T::type_name());

    return result;
}

template <typename T>
const T& RpcReplyDataType<T>::data() const
{
    return data_;
}

template <typename T>
void RpcReplyDataType<T>::data(
        T new_value)
{
    data_ = new_value;
}

template <typename T>
std::ostream& operator <<(
        std::ostream& os,
        const RpcReplyDataType<T>& reply)
{
    os << "RPC-REPLY{" << reply.server_id() << "|" << reply.task_id() << "}";
    return os;
}

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

#endif // AMLIPCPP__SRC_CPP_TYPES_IMPL_RPCREPLYDATATYPE_IPP
