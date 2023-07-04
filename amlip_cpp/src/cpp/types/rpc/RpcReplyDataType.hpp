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
 * @file RpcReplyDataType.hpp
 */

#ifndef AMLIPCPP_TYPES_RPCREPLYDATATYPE_HPP
#define AMLIPCPP_TYPES_RPCREPLYDATATYPE_HPP

#include <array>
#include <limits>
#include <ostream>
#include <string>

#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <amlip_cpp/types/id/TaskId.hpp>
#include <amlip_cpp/types/InterfaceDataType.hpp>

namespace eprosima {
namespace fastcdr {
// Forward declaration of the CDR class
class Cdr;
} // namespace fastcdr
} // namespace eprosima

namespace eprosima {
namespace amlip {
namespace types {

/**
 * TODO
 */
template <typename T>
class RpcReplyDataType : public InterfaceDataType
{

    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, T);

public:

    RpcReplyDataType();

    RpcReplyDataType(
            const AmlipIdDataType& client_id,
            const TaskId& task_id,
            const AmlipIdDataType& server_id,
            const T& data);

    /*!
     * @brief Default destructor.
     */
    virtual ~RpcReplyDataType();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object RpcReplyDataType that will be copied.
     */
    RpcReplyDataType(
            const RpcReplyDataType& x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object RpcReplyDataType that will be copied.
     */
    RpcReplyDataType(
            RpcReplyDataType&& x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object RpcReplyDataType that will be copied.
     */
    RpcReplyDataType& operator =(
            const RpcReplyDataType& x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object RpcReplyDataType that will be copied.
     */
    RpcReplyDataType& operator =(
            RpcReplyDataType&& x);

    /*!
     * @brief Comparison operator.
     * @param x RpcReplyDataType object to compare.
     */
    bool operator ==(
            const RpcReplyDataType& x) const;

    /*!
     * @brief Comparison operator.
     * @param x RpcReplyDataType object to compare.
     */
    bool operator !=(
            const RpcReplyDataType& x) const;

    /*!
     * @brief Comparison operator.
     * @param x RpcReplyDataType object to compare.
     */
    bool operator <(
            const RpcReplyDataType& x) const;

    /*!
     * TODO
     */
    AmlipIdDataType client_id() const;

    /*!
     * TODO
     */
    void client_id(
            const AmlipIdDataType& new_value);

    /*!
     * TODO
     */
    TaskId task_id() const;

    /*!
     * TODO
     */
    void task_id(
            const TaskId& new_value);

    /*!
     * TODO
     */
    AmlipIdDataType server_id() const;

    /*!
     * TODO
     */
    void server_id(
            const AmlipIdDataType& new_value);

    /////
    // InterfaceDataType methods

    /*!
     * @brief This function serializes an object using CDR serialization.
     *
     * @param cdr CDR serialization object.
     *
     * @warning this method must be overriden in child class.
     */
    virtual void serialize(
            eprosima::fastcdr::Cdr& cdr) const override;

    /*!
     * @brief This function deserializes an object using CDR serialization.
     *
     * @param cdr CDR serialization object.
     *
     * @warning this method must be overriden in child class.
     */
    virtual void deserialize(
            eprosima::fastcdr::Cdr& cdr) override;

    /*!
     * @brief This function serializes the key members of an object using CDR serialization.
     *
     * @param cdr CDR serialization object.
     *
     * @warning this method must be overriden in child class.
     */
    virtual void serialize_key(
            eprosima::fastcdr::Cdr& cdr) const override;

    /*!
     * @brief This function returns the maximum serialized size of an object
     * depending on the buffer alignment.
     *
     * @param current_alignment Buffer alignment.
     *
     * @return Maximum serialized size.
     *
     * @warning this method must be overriden in child class.
     */
    static size_t get_max_cdr_serialized_size(
            size_t current_alignment = 0);

    /*!
     * @brief This function returns the serialized size of a data depending on the buffer alignment.

     * @param data Data which is calculated its serialized size.
     * @param current_alignment Buffer alignment.
     *
     * @return Serialized size.
     *
     * @warning this method must be overriden in child class.
     */
    static size_t get_cdr_serialized_size(
            const RpcReplyDataType& data,
            size_t current_alignment = 0);

    /*!
     * @brief This function returns the maximum serialized size of the Key of an object
     * depending on the buffer alignment.
     *
     * @param current_alignment Buffer alignment.
     *
     * @return Maximum serialized size.
     *
     * @warning this method must be overriden in child class.
     */
    static size_t get_key_max_cdr_serialized_size(
            size_t current_alignment = 0);

    /*!
     * @brief This function tells you if the Key has been defined for this type
     *
     * @warning this method must be overriden in child class.
     */
    static bool is_key_defined();

    /**
     * @brief Whether the type is bounded
     *
     * @warning this method must be overriden in child class.
     */
    static bool is_bounded();

    /**
     * @brief Whether the type is plain
     *
     * @warning this method must be overriden in child class.
     */
    static bool is_plain();

    /**
     * @brief Construct an empty sample in the memory allocated
     *
     * @pre The type must be plain
     *
     * @param memory already allocated memory for the new data
     *
     * @return true if the construction was successful, false otherwise
     *
     * @warning this method must be overriden in child class.
     */
    static bool construct_sample(
            void* memory);

    /**
     * @brief Name of the Data Type. This name will be used as the DDS type name.
     *
     * @warning this method must be overriden in child class.
     */
    static std::string type_name();

    const T& data() const;

    void data(
            T new_value);

protected:

    AmlipIdDataType client_id_;

    TaskId task_id_;

    AmlipIdDataType server_id_;

    static const char* DATA_TYPE_PREFIX_NAME_;

    T data_;
};

//! \c RpcReplyDataType to stream serializator
template <typename T>
std::ostream& operator <<(
        std::ostream& os,
        const RpcReplyDataType<T>& reply);

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <types/rpc/impl/RpcReplyDataType.ipp>

#endif // AMLIPCPP_TYPES_RPCREPLYDATATYPE_HPP
