// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file GenericType.hpp
 * This header file contains the declaration of a generic type that contains void* data.
 */

#ifndef AMLIP__SRC_CPP_AMLIPTYPES_GENERICTYPE_HPP
#define AMLIP__SRC_CPP_AMLIPTYPES_GENERICTYPE_HPP

#include <atomic>

#include <amlip_node/types/InterfaceDataType.hpp>

namespace eprosima {
namespace fastcdr {
class Cdr;
} // namespace fastcdr
} // namespace eprosima


namespace eprosima {
namespace amlip {
namespace types {

/*!
 * @brief This class represents the structure GenericType defined by the user in the IDL file.
 * @ingroup AMLIP
 */
class GenericType : public InterfaceDataType
{
public:

    /*!
     * @brief Default constructor with values.
     */
    GenericType(
        void* data,
        const uint32_t size);

    /*!
     * @brief Default constructor.
     */
    GenericType();

    /*!
     * @brief Default destructor.
     */
    ~GenericType();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object GenericType that will be copied.
     */
    GenericType(
            const GenericType& x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object GenericType that will be copied.
     */
    GenericType(
            GenericType&& x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object GenericType that will be copied.
     */
    GenericType& operator =(
            const GenericType& x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object GenericType that will be copied.
     */
    GenericType& operator =(
            GenericType&& x);

    /*!
     * @brief Comparison operator.
     * @param x GenericType object to compare.
     */
    bool operator ==(
            const GenericType& x) const;

    /*!
     * @brief Comparison operator.
     * @param x GenericType object to compare.
     */
    bool operator !=(
            const GenericType& x) const;

    // TODO comment
    void* data() const;

    // TODO comment
    uint32_t data_size() const;

    /*!
     * @brief This function returns the maximum serialized size of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    static size_t getMaxCdrSerializedSize(
            size_t current_alignment = 0);

    /*!
     * @brief This function returns the serialized size of a data depending on the buffer alignment.
     * @param data Data which is calculated its serialized size.
     * @param current_alignment Buffer alignment.
     * @return Serialized size.
     */
    static size_t getCdrSerializedSize(
            const GenericType& data,
            size_t current_alignment = 0);


    /*!
     * @brief This function serializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    void serialize(
            eprosima::fastcdr::Cdr& cdr) const override;

    /*!
     * @brief This function deserializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    void deserialize(
            eprosima::fastcdr::Cdr& cdr) override;



    /*!
     * @brief This function returns the maximum serialized size of the Key of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    static size_t getKeyMaxCdrSerializedSize(
            size_t current_alignment = 0);

    /*!
     * @brief This function tells you if the Key has been defined for this type
     */
    static bool isKeyDefined();

    /*!
     * @brief This function serializes the key members of an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    void serializeKey(
            eprosima::fastcdr::Cdr& cdr) const override;

    static bool is_bounded();

    static bool is_plain();

    static bool construct_sample(
            void* memory);

    static const char* type_name();

protected:

    void* data_;

    uint32_t data_size_;

    std::atomic<bool> is_been_allocated_;
};

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

#endif // AMLIP__SRC_CPP_AMLIPTYPES_GENERICTYPE_HPP