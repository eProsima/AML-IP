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
#include <string>
#include <vector>

#include <fastcdr/config.h>

#include <amlip_cpp/library/library_dll.h>
#include <amlip_cpp/types/InterfaceDataType.hpp>


namespace eprosima {
namespace amlip {
namespace types {

using ByteType = uint8_t;

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
    AMLIP_CPP_DllAPI GenericDataType(
            void* data,
            const uint32_t size,
            bool take_ownership = false);

    /*!
     * @brief Default constructor.
     */
    AMLIP_CPP_DllAPI GenericDataType();

    AMLIP_CPP_DllAPI GenericDataType(
            const std::vector<ByteType>& bytes);

    AMLIP_CPP_DllAPI GenericDataType(
            const std::string& bytes);

    /*!
     * @brief Default destructor.
     */
    AMLIP_CPP_DllAPI virtual ~GenericDataType();

    /*!
     * @brief Copy constructor.
     *
     * If \c x has no ownership, it copies the reference.
     * If \c x has ownership, it copies the data inside.
     */
    AMLIP_CPP_DllAPI GenericDataType(
            const GenericDataType& x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object GenericDataType that will be copied.
     */
    AMLIP_CPP_DllAPI GenericDataType(
            GenericDataType&& x);

    /*!
     * @brief Copy assignment not allowed.
     * To be defined by end-user, as copying \c data_ pointer or its content may be preferred depending on the scenario.
     */
    AMLIP_CPP_DllAPI GenericDataType& operator =(
            const GenericDataType&);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object GenericDataType that will be copied.
     */
    AMLIP_CPP_DllAPI GenericDataType& operator =(
            GenericDataType&& x);

    /*!
     * @brief Comparison operator.
     * @param x GenericDataType object to compare.
     *
     * @note \c data_ attributes are compared as pointers only, i.e. the content pointed to is not compared.
     */
    AMLIP_CPP_DllAPI bool operator ==(
            const GenericDataType& x) const;

    /*!
     * @brief Comparison operator.
     * @param x GenericDataType object to compare.
     *
     * @note \c data_ attributes are compared as pointers only, i.e. the content pointed to is not compared.
     */
    AMLIP_CPP_DllAPI bool operator !=(
            const GenericDataType& x) const;

    /*!
     * @brief This function copies the value in member \c data_
     * @param data New value to be copied in member id \c data_
     */
    AMLIP_CPP_DllAPI void data(
            void* data);

    /*!
     * @brief Returns value of attribute \c data_
     */
    AMLIP_CPP_DllAPI void* data() const;

    /*!
     * @brief Returns value of attribute \c data_size_
     */
    AMLIP_CPP_DllAPI uint32_t data_size() const;

    /*!
     * @brief Returns reference to attribute \c data_size_
     */
    AMLIP_CPP_DllAPI uint32_t& data_size();

    /*!
     * @brief This function returns the name of this specific data type
     */
    AMLIP_CPP_DllAPI static std::string type_name();

    /*!
     * @brief This function returns value of attribute \c has_been_allocated_
     */
    AMLIP_CPP_DllAPI bool has_been_allocated() const;

    /*!
     * @brief This function copies the value in member \c has_been_allocated_
     * @param take_ownership New value to be copied in member \c has_been_allocated_
     */
    AMLIP_CPP_DllAPI void has_been_allocated(
            bool take_ownership);

    /////
    // InterfaceDataType methods

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

    AMLIP_CPP_DllAPI std::string to_string() const noexcept;

    AMLIP_CPP_DllAPI std::vector<ByteType> to_vector() const noexcept;

    AMLIP_CPP_DllAPI static uint32_t max_cdr_typesize_;

    static constexpr uint32_t max_key_cdr_typesize_ {0UL};

protected:

    void* data_;

    uint32_t data_size_;

    std::atomic<bool> has_been_allocated_;

    static const char* TYPE_NAME_;

    static const size_t DEFAULT_PREALLOCATED_SIZE_;
};

//! \c GenericDataType to stream serializator
std::ostream& operator <<(
        std::ostream& os,
        const GenericDataType& obj);

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

#endif // AMLIP__SRC_CPP_TYPES_GENERICDATATYPE_HPP
