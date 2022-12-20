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
 * @file MsReferenceDataType.hpp
 */

#ifndef AMLIP__SRC_CPP_TYPES_MSREFERENCEDATATYPE_HPP
#define AMLIP__SRC_CPP_TYPES_MSREFERENCEDATATYPE_HPP

#include <array>
#include <limits>
#include <ostream>
#include <string>

#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <types/multiservice/MsRequestDataType.hpp>

namespace eprosima {
namespace fastcdr {
class Cdr;
} // namespace fastcdr
} // namespace eprosima

namespace eprosima {
namespace amlip {
namespace types {

/*!
 * TODO
 */
class MsReferenceDataType : public MsRequestDataType
{
public:

    /**
     * TODO
     */
    MsReferenceDataType();

    /*!
     * @brief Constructor with name.
     */
    MsReferenceDataType(
            const AmlipIdDataType client_id,
            const TaskId& task_id,
            const AmlipIdDataType& server_id);

    /*!
     * @brief Default destructor.
     */
    virtual ~MsReferenceDataType();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object MsReferenceDataType that will be copied.
     */
    MsReferenceDataType(
            const MsReferenceDataType& x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object MsReferenceDataType that will be copied.
     */
    MsReferenceDataType(
            MsReferenceDataType&& x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object MsReferenceDataType that will be copied.
     */
    MsReferenceDataType& operator =(
            const MsReferenceDataType& x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object MsReferenceDataType that will be copied.
     */
    MsReferenceDataType& operator =(
            MsReferenceDataType&& x);

    /*!
     * @brief Comparison operator.
     * @param x MsReferenceDataType object to compare.
     */
    bool operator ==(
            const MsReferenceDataType& x) const;

    /*!
     * @brief Comparison operator.
     * @param x MsReferenceDataType object to compare.
     */
    bool operator !=(
            const MsReferenceDataType& x) const;

    /*!
     * @brief Comparison operator.
     * @param x MsReferenceDataType object to compare.
     */
    bool operator <(
            const MsReferenceDataType& x) const;

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
     * @brief This function serializes the key members of an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    void serialize_key(
            eprosima::fastcdr::Cdr& cdr) const override;

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
            const MsReferenceDataType& data,
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

    /**
     * @brief Name of the Data Type. This name will be used as the DDS type name.
     *
     * @warning this method must be overriden in child class.
     */
    static std::string type_name();

    //! Return the Request related with this reference
    MsRequestDataType request() const;

protected:

    AmlipIdDataType server_id_;

    static const char* DATA_TYPE_NAME_; // "ms_reference"
};

//! \c MsReferenceDataType to stream serializator
std::ostream& operator <<(
        std::ostream& os,
        const MsReferenceDataType& reference);

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

#endif // AMLIP__SRC_CPP_TYPES_MSREFERENCEDATATYPE_HPP
