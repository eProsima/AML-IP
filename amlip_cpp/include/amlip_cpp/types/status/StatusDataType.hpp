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
 * @file StatusDataType.hpp
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool fastddsgen and refactored by a proto-intelligent try-hard human.
 */

#ifndef AMLIPCPP__SRC_CPP_TYPES_STATUSDATATYPE_HPP
#define AMLIPCPP__SRC_CPP_TYPES_STATUSDATATYPE_HPP

#include <fastcdr/config.h>

#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <amlip_cpp/types/InterfaceDataType.hpp>
#include <amlip_cpp/types/status/NodeKind.hpp>
#include <amlip_cpp/types/status/StateKind.hpp>

namespace eprosima {
namespace amlip {
namespace types {

/*!
 * @brief This class represents the structure StatusDataType defined by the user in the IDL file.
 * @ingroup STATUS
 */
class StatusDataType : public InterfaceDataType
{
public:

    /*!
     * @brief Default constructor.
     */
    AMLIP_CPP_DllAPI StatusDataType();

    /*!
     * @brief Default constructor.
     */
    AMLIP_CPP_DllAPI StatusDataType(
            const AmlipIdDataType& id,
            const NodeKind& node_kind,
            const StateKind& state);

    AMLIP_CPP_DllAPI virtual ~StatusDataType() = default;

    /*!
     * @brief Comparison operator.
     * @param x StatusDataType object to compare.
     */
    AMLIP_CPP_DllAPI bool operator ==(
            const StatusDataType& x) const;

    /*!
     * @brief Comparison operator.
     * @param x StatusDataType object to compare.
     */
    AMLIP_CPP_DllAPI bool operator !=(
            const StatusDataType& x) const;

    //! Get id value
    AMLIP_CPP_DllAPI AmlipIdDataType id() const noexcept;

    //! Get reference id value
    AMLIP_CPP_DllAPI AmlipIdDataType& id() noexcept;

    //! Get Node Kind value
    AMLIP_CPP_DllAPI NodeKind node_kind() const noexcept;

    //! Get reference Node Kind value
    AMLIP_CPP_DllAPI NodeKind& node_kind() noexcept;

    //! Get State Kind value
    AMLIP_CPP_DllAPI StateKind state() const noexcept;

    //! Get reference State Kind value
    AMLIP_CPP_DllAPI StateKind& state() noexcept;

    //! Whether this object is correctly defined, thus none of its internal variables are undefined.
    AMLIP_CPP_DllAPI bool is_defined() const noexcept;

    //! Overload parent \c to_string method
    AMLIP_CPP_DllAPI std::string to_string() const noexcept;

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

    /**
     * @brief Name of the Data Type. This name will be used as the DDS type name.
     *
     * @warning this method must be overriden in child class.
     */
    AMLIP_CPP_DllAPI static std::string type_name();

    static constexpr uint32_t max_cdr_typesize_ {140UL};

    static constexpr uint32_t max_key_cdr_typesize_ {0UL};

protected:

    //! Internal variable Id
    AmlipIdDataType id_;

    //! Internal variable Node Kind
    NodeKind node_kind_;

    //! Internal variable State Kind
    StateKind state_;

    static const char* TYPE_NAME_;

};

//! \c StatusDataType to stream serializator
AMLIP_CPP_DllAPI std::ostream& operator <<(
        std::ostream& os,
        const StatusDataType& st);

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

#endif // AMLIPCPP__SRC_CPP_TYPES_STATUSDATATYPE_HPP
