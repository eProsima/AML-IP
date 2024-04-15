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

#include <fastcdr/config.h>

#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <amlip_cpp/types/id/TaskId.hpp>
#include <amlip_cpp/types/InterfaceDataType.hpp>


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
    AmlipIdDataType& client_id();

    /*!
     * TODO
     */
    void client_id(
            AmlipIdDataType& new_value);

    /*!
     * TODO
     */
    TaskId task_id() const;

    /*!
     * TODO
     */
    TaskId& task_id();

    /*!
     * TODO
     */
    void task_id(
            TaskId& new_value);

    /////
    // InterfaceDataType methods

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

    static constexpr uint32_t max_cdr_typesize_ {136UL};

    static constexpr uint32_t max_key_cdr_typesize_ {0UL};

protected:

    AmlipIdDataType client_id_;

    TaskId task_id_;

    static const char* TYPE_NAME_; // "ms_request"

};

//! \c MsRequestDataType to stream serializator
std::ostream& operator <<(
        std::ostream& os,
        const MsRequestDataType& request);

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

#endif // AMLIP__SRC_CPP_TYPES_MSREQUESTDATATYPE_HPP
