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
 * @file GenericDataType.hpp
 * This header file contains the declaration of a generic type that contains void* data.
 */

#ifndef AMLIP__SRC_CPP_TYPES_GENERICDATATYPE_HPP
#define AMLIP__SRC_CPP_TYPES_GENERICDATATYPE_HPP

#include <atomic>

#include <types/InterfaceDataType.hpp>

namespace eprosima {
namespace fastcdr {
class Cdr;
} // namespace fastcdr
} // namespace eprosima


namespace eprosima {
namespace amlip {
namespace types {

/*!
 * @brief This class represents the structure GenericDataType, which allows to send any type of data as a void*.
 * This is, data is streamed as an array of octets given its size in memory regardless of its original type/structure.
 * @ingroup AMLIP
 */
class GenericDataType : public InterfaceDataType
{
public:

    /*!
     * @brief Default constructor with values.
     */
    GenericDataType(
            void* data,
            const uint32_t size,
            bool take_ownership = false);

    /*!
     * @brief Default constructor.
     */
    GenericDataType();

    /*!
     * @brief Default destructor.
     */
    virtual ~GenericDataType();

    /*!
     * @brief Copy constructor.
     *
     * If \c x has no ownership, it copies the reference.
     * If \c x has ownership, it copies the data inside.
     */
    GenericDataType(
            const GenericDataType& x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object GenericDataType that will be copied.
     */
    GenericDataType(
            GenericDataType&& x);

    /*!
     * @brief Copy assignment not allowed.
     * To be defined by end-user, as copying \c data_ pointer or its content may be preferred depending on the scenario.
     */
    GenericDataType& operator =(
            const GenericDataType&) = delete;

    /*!
     * @brief Move assignment.
     * @param x Reference to the object GenericDataType that will be copied.
     */
    GenericDataType& operator =(
            GenericDataType&& x);

    /*!
     * @brief Comparison operator.
     * @param x GenericDataType object to compare.
     *
     * @note \c data_ attributes are compared as pointers only, i.e. the content pointed to is not compared.
     */
    bool operator ==(
            const GenericDataType& x) const;

    /*!
     * @brief Comparison operator.
     * @param x GenericDataType object to compare.
     *
     * @note \c data_ attributes are compared as pointers only, i.e. the content pointed to is not compared.
     */
    bool operator !=(
            const GenericDataType& x) const;

    /*!
     * @brief Return value of attribute \c data_
     */
    void* data() const;

    /*!
     * @brief Return value of attribute \c data__size_
     */
    uint32_t data_size() const;

    /*!
     * @brief This function returns the name of this specific data type
     */
    static std::string type_name();

    /////
    // InterfaceDataType methods

    /*!
     * @brief This function serializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    void serialize(
            eprosima::fastcdr::Cdr& cdr) const;

    /*!
     * @brief This function deserializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    void deserialize(
            eprosima::fastcdr::Cdr& cdr);

    /*!
     * @brief This function serializes the key members of an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    void serialize_key(
            eprosima::fastcdr::Cdr& cdr) const;

    /*!
     * @brief This function returns the maximum serialized size of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    static size_t get_max_cdr_serialized_size(
            size_t current_alignment = 0);

    /*!
     * @brief This function returns the serialized size of a data depending on the buffer alignment.
     * @param data Data which is calculated its serialized size.
     * @param current_alignment Buffer alignment.
     * @return Serialized size.
     */
    static size_t get_cdr_serialized_size(
            const GenericDataType& data,
            size_t current_alignment = 0);

    /*!
     * @brief This function returns the maximum serialized size of the Key of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    static size_t get_key_max_cdr_serialized_size(
            size_t current_alignment = 0);

    /*!
     * @brief This function tells you if the Key has been defined for this type
     */
    static bool is_key_defined();

    /**
     * @brief Whether the type is bounded
     */
    static bool is_bounded();

    /**
     * @brief Whether the type is plain
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
     */
    static bool construct_sample(
            void* memory);

protected:

    void* data_;

    uint32_t data_size_;

    std::atomic<bool> has_been_allocated_;

    static const char* TYPE_NAME_;

    static const size_t DEFAULT_PREALLOCATED_SIZE_;
};

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

#endif // AMLIP__SRC_CPP_TYPES_GENERICDATATYPE_HPP
