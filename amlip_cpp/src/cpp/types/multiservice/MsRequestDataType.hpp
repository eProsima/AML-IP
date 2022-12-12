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
 * @file MsRequestDataType.hpp
 */

#ifndef AMLIP__SRC_CPP_TYPES_MSREQUESTDATATYPE_HPP
#define AMLIP__SRC_CPP_TYPES_MSREQUESTDATATYPE_HPP

#include <array>
#include <limits>
#include <ostream>
#include <string>

#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <amlip_cpp/types/id/TaskId.hpp>
#include <amlip_cpp/types/InterfaceDataType.hpp>

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
class MsRequestDataType : public InterfaceDataType
{
public:

    /**
     * TODO
     */
    MsRequestDataType();

    /*!
     * @brief Constructor with name.
     */
    MsRequestDataType(
            const AmlipIdDataType client_id,
            const TaskId& task_id);

    /*!
     * @brief Default destructor.
     */
    virtual ~MsRequestDataType();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object MsRequestDataType that will be copied.
     */
    MsRequestDataType(
            const MsRequestDataType& x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object MsRequestDataType that will be copied.
     */
    MsRequestDataType(
            MsRequestDataType&& x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object MsRequestDataType that will be copied.
     */
    MsRequestDataType& operator =(
            const MsRequestDataType& x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object MsRequestDataType that will be copied.
     */
    MsRequestDataType& operator =(
            MsRequestDataType&& x);

    /*!
     * @brief Comparison operator.
     * @param x MsRequestDataType object to compare.
     */
    bool operator ==(
            const MsRequestDataType& x) const;

    /*!
     * @brief Comparison operator.
     * @param x MsRequestDataType object to compare.
     */
    bool operator !=(
            const MsRequestDataType& x) const;

    /*!
     * @brief Comparison operator.
     * @param x MsRequestDataType object to compare.
     */
    bool operator <(
            const MsRequestDataType& x) const;

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
            const MsRequestDataType& data,
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

protected:

    AmlipIdDataType client_id_;

    TaskId task_id_;

    static const char* DATA_TYPE_NAME_; // "ms_request"

};

//! \c MsRequestDataType to stream serializator
std::ostream& operator <<(
        std::ostream& os,
        const MsRequestDataType& request);

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

#endif // AMLIP__SRC_CPP_TYPES_MSREQUESTDATATYPE_HPP
