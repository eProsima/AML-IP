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
 * @file InterfaceDataType.hpp
 */

#ifndef AMLIP__SRC_CPP_AMLIPTYPES_INTERFACEDATATYPE_HPP
#define AMLIP__SRC_CPP_AMLIPTYPES_INTERFACEDATATYPE_HPP

#include <string>

namespace eprosima {
namespace fastcdr {
class Cdr;
} // namespace fastcdr
} // namespace eprosima

namespace eprosima {
namespace amlip {
namespace types {

class InterfaceDataType
{
public:

    /*!m
     * @brief This function serializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    virtual void serialize(
            eprosima::fastcdr::Cdr& cdr) const = 0;

    /*!
     * @brief This function deserializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    virtual void deserialize(
            eprosima::fastcdr::Cdr& cdr) = 0;


    /*!
     * @brief This function serializes the key members of an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    virtual void serializeKey(
            eprosima::fastcdr::Cdr& cdr) const = 0;

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
            const InterfaceDataType& data,
            size_t current_alignment = 0);

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

    static bool is_bounded();

    static bool is_plain();

    static bool construct_sample(
            void* memory);

    static const char* type_name();
};

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

#endif // AMLIP__SRC_CPP_AMLIPTYPES_INTERFACEDATATYPE_HPP