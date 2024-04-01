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
 * @file AmlipIdDataType.hpp
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool fastddsgen and refactored by a try-hard human.
 */

#ifndef AMLIP__SRC_CPP_TYPES_AMLIPIDDATATYPE_HPP
#define AMLIP__SRC_CPP_TYPES_AMLIPIDDATATYPE_HPP

#include <cstdint>
#include <array>
#include <limits>
#include <ostream>
#include <string>

#include <fastcdr/config.h>

#include <amlip_cpp/types/InterfaceDataType.hpp>


namespace eprosima {
namespace amlip {
namespace types {

//! Number of octets conforming the alphanumerical identifier \c name_
constexpr const uint32_t NAME_SIZE = 28;
//! Number of octets conforming the random numerical identifier \c rand_id_
constexpr const uint32_t RAND_SIZE = 4;

/*!
 * @brief This class represents the structure AmlipIdDataType, which implements \c InterfaceDataType and hence serves
 * as a communication interface to \c AmlipId class.
 * @ingroup AMLIP
 */
class AmlipIdDataType : public InterfaceDataType
{
public:

    /*!
     * @brief Default constructor.
     */
    AMLIP_CPP_DllAPI AmlipIdDataType();

    /*!
     * @brief Constructor with name.
     */
    AMLIP_CPP_DllAPI AmlipIdDataType(
            const std::string& name);

    /*!
     * @brief Constructor with name given in char*.
     */
    AMLIP_CPP_DllAPI AmlipIdDataType(
            const char* name);

    /*!
     * @brief Constructor with name and random id given as arrays of octets.
     */
    AMLIP_CPP_DllAPI AmlipIdDataType(
            const std::array<uint8_t, NAME_SIZE>& name,
            const std::array<uint8_t, RAND_SIZE>& rand_id);

    /*!
     * @brief Constructor given fields.
     *
     * The parameters must be moved, as they will taken as part of the new object.
     */
    AMLIP_CPP_DllAPI AmlipIdDataType(
            std::array<uint8_t, NAME_SIZE>&& name,
            std::array<uint8_t, RAND_SIZE>&& rand_id);

    /*!
     * @brief Default destructor.
     */
    AMLIP_CPP_DllAPI virtual ~AmlipIdDataType();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object AmlipIdDataType that will be copied.
     */
    AMLIP_CPP_DllAPI AmlipIdDataType(
            const AmlipIdDataType& x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object AmlipIdDataType that will be copied.
     */
    AMLIP_CPP_DllAPI AmlipIdDataType(
            AmlipIdDataType&& x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object AmlipIdDataType that will be copied.
     */
    AMLIP_CPP_DllAPI AmlipIdDataType& operator =(
            const AmlipIdDataType& x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object AmlipIdDataType that will be copied.
     */
    AMLIP_CPP_DllAPI AmlipIdDataType& operator =(
            AmlipIdDataType&& x);

    /*!
     * @brief Comparison operator.
     * @param x AmlipIdDataType object to compare.
     */
    AMLIP_CPP_DllAPI bool operator ==(
            const AmlipIdDataType& x) const;

    /*!
     * @brief Comparison operator.
     * @param x AmlipIdDataType object to compare.
     */
    AMLIP_CPP_DllAPI bool operator !=(
            const AmlipIdDataType& x) const;

    /*!
     * @brief Comparison operator.
     * @param x AmlipIdDataType object to compare.
     */
    AMLIP_CPP_DllAPI bool operator <(
            const AmlipIdDataType& x) const;

    /*!
     * @brief This function gets the value of member \c name_ as string
     * @return Value of member \c name_ as string
     */
    AMLIP_CPP_DllAPI std::string name() const;

    /*!
     * @brief This function copies the value in member \c name_
     * @param name New value to be copied in member id \c name_
     */
    AMLIP_CPP_DllAPI void name(
            const std::array<uint8_t, NAME_SIZE>& name);

    /*!
     * @brief This function copies the value in member \c name_
     * @param name New value to be copied in member id \c name_
     */
    AMLIP_CPP_DllAPI void name(
            const std::string& name);

    /*!
     * @brief This function gets the value in member \c name as array of octets
     * @return Value of member \c name_ as array of octets
     */
    AMLIP_CPP_DllAPI std::array<uint8_t, NAME_SIZE> base64_name() const;

    /*!
     * @brief This function gets the value in member \c rand_id as array of octets
     * @return Value of member \c rand_id_ as array of octets
     */
    AMLIP_CPP_DllAPI std::array<uint8_t, RAND_SIZE> id() const;

    /*!
     * @brief This function returns reference to \c rand_id_
     * @return Reference to \c rand_id_
     */
    AMLIP_CPP_DllAPI std::array<uint8_t, RAND_SIZE>& id();

    /*!
     * @brief This function copies the value in member \c rand_id_
     * @param id New value to be copied in member id \c rand_id_
     */
    AMLIP_CPP_DllAPI void id(
            const std::array<uint8_t, RAND_SIZE>& id);

    /*!
     * @brief This function creates a string that uniquely describes this object.
     *
     * @note This string is forced to be valid as a DDS name for entity or topic.
     */
    AMLIP_CPP_DllAPI std::string to_dds_string() const;

    /*!
     * @brief This function returns the name of this specific data type
     */
    AMLIP_CPP_DllAPI static std::string type_name();

    /*!
     * @brief This function returns true if the object is defined (i.e. constructed with a specific name)
     */
    AMLIP_CPP_DllAPI bool is_defined() const noexcept;

    //! Override parent \c to_string method using << operator
    AMLIP_CPP_DllAPI std::string to_string() const noexcept;

    /*!
     * @brief This function returns an \c AmlipIdDataType object with random name and numerical id
     */
    AMLIP_CPP_DllAPI static AmlipIdDataType new_unique_id();

    /*!
     * @brief This function returns an \c AmlipIdDataType object with the given name and a random numerical id
     * @param name Value given as string to set in attribute \c name_
     */
    AMLIP_CPP_DllAPI static AmlipIdDataType new_unique_id(
            const std::string& name);

    /*!
     * @brief This function returns an \c AmlipIdDataType object with the given name and a random numerical id
     * @param name Value given as char* to set in attribute \c name_
     */
    AMLIP_CPP_DllAPI static AmlipIdDataType new_unique_id(
            const char* name);

    /*!
     * @brief This function returns an \c AmlipIdDataType object corresponding to an undefined \c AmlipDataType
     */
    AMLIP_CPP_DllAPI static AmlipIdDataType undefined_id();

    /////
    // InterfaceDataType methods

    /*!
     * @brief This function serializes the key members of an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    AMLIP_CPP_DllAPI void serialize_key(
            eprosima::fastcdr::Cdr& cdr) const override;

    /*!
     * @brief This function returns the maximum serialized size of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    AMLIP_CPP_DllAPI static size_t get_max_cdr_serialized_size(
            size_t current_alignment = 0);

    /*!
     * @brief This function returns the serialized size of a data depending on the buffer alignment.
     * @param data Data which is calculated its serialized size.
     * @param current_alignment Buffer alignment.
     * @return Serialized size.
     */
    AMLIP_CPP_DllAPI static size_t get_cdr_serialized_size(
            const AmlipIdDataType& data,
            size_t current_alignment = 0);

    /*!
     * @brief This function returns the maximum serialized size of the Key of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    AMLIP_CPP_DllAPI static size_t get_key_max_cdr_serialized_size(
            size_t current_alignment = 0);

    /*!
     * @brief This function tells you if the Key has been defined for this type
     */
    AMLIP_CPP_DllAPI static bool is_key_defined();

    /**
     * @brief Whether the type is bounded
     */
    AMLIP_CPP_DllAPI static bool is_bounded();

    /**
     * @brief Whether the type is plain
     */
    AMLIP_CPP_DllAPI static bool is_plain();

    /**
     * @brief Construct an empty sample in the memory allocated
     *
     * @pre The type must be plain
     *
     * @param memory already allocated memory for the new data
     *
     * @return true if the construction was successful, false otherwise
     */
    AMLIP_CPP_DllAPI static bool construct_sample(
            void* memory);

protected:

    std::array<uint8_t, NAME_SIZE> name_;

    std::array<uint8_t, RAND_SIZE> rand_id_;

    static const AmlipIdDataType UNDEFINED_ID_;

    static const char* TYPE_NAME_;

    /*!
     * @brief This function converts a char* to a vector of octets of fixed size, trimming its end if too long and
     * padding with null characters when too short.
     * @param name Value to be converted given as char*
     * @return Converted value to a fixed-length array of octets
     */
    AMLIP_CPP_DllAPI static std::array<uint8_t, NAME_SIZE> char_name_to_array_(
            const char* name);

    /*!
     * @brief This function converts a string to a vector of octets of fixed size, trimming its end if too long and
     * padding with null characters when too short.
     * @param name Value to be converted given as string
     * @return Converted value to a fixed-length array of octets
     */
    AMLIP_CPP_DllAPI static std::array<uint8_t, NAME_SIZE> str_name_to_array_(
            const std::string& name);

    /*!
     * @brief This function converts a vector of octets of fixed size to a string.
     * @param name Value to be converted given as vector of octets of fixed size
     * @return Converted value to a string
     */
    AMLIP_CPP_DllAPI static std::string array_name_to_str_(
            const std::array<uint8_t, NAME_SIZE>& name);

    /*!
     * @brief This function generates a random name as a fixed-length array of octets
     */
    AMLIP_CPP_DllAPI static std::array<uint8_t, NAME_SIZE> random_name_();

    /*!
     * @brief This function generates a random numerical identifier as a fixed-length array of octets
     */
    AMLIP_CPP_DllAPI static std::array<uint8_t, RAND_SIZE> random_id_();
};

//! \c AmlipIdDataType to stream serializator
AMLIP_CPP_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const AmlipIdDataType& id);

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

#endif // AMLIP__SRC_CPP_TYPES_AMLIPIDDATATYPE_HPP
