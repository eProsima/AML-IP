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
 * @file InterfaceDataType.hpp
 */

#ifndef AMLIPCPP_TYPES_INTERFACEDATATYPE_HPP
#define AMLIPCPP_TYPES_INTERFACEDATATYPE_HPP

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
 * @brief Interface to create DDS DataTypes.
 *
 * Every DataType that will work beneath the AMLIP library must implement this interface.
 * This is because the "PubSub" type (actually TopicDataType) is a template class, and it needs
 * to know the type of the data that will be published.
 * This specialization could only be done from a class inheriting from this interface.
 *
 * @warning This interface requires to override some methods, including static ones.
 * Every class inheriting from it must override every method initialized in this file.
 */
class InterfaceDataType
{
public:

    /*!
     * @brief This function serializes an object using CDR serialization.
     *
     * @param cdr CDR serialization object.
     *
     * @warning this method must be overriden in child class.
     */
    virtual void serialize(
            eprosima::fastcdr::Cdr& cdr) const = 0;

    /*!
     * @brief This function deserializes an object using CDR serialization.
     *
     * @param cdr CDR serialization object.
     *
     * @warning this method must be overriden in child class.
     */
    virtual void deserialize(
            eprosima::fastcdr::Cdr& cdr) = 0;


    /*!
     * @brief This function serializes the key members of an object using CDR serialization.
     *
     * @param cdr CDR serialization object.
     *
     * @warning this method must be overriden in child class.
     */
    virtual void serialize_key(
            eprosima::fastcdr::Cdr& cdr) const = 0;

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
            const InterfaceDataType& data,
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
    static const char* type_name();
};

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

#endif // AMLIPCPP_TYPES_INTERFACEDATATYPE_HPP
