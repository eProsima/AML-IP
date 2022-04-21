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
 * @file AmlipId.hpp
 */

#ifndef AMLIP_AMLIPCPP_TYPES_AMLIPID_HPP
#define AMLIP_AMLIPCPP_TYPES_AMLIPID_HPP

#include <memory>
#include <ostream>
#include <string>

namespace eprosima {
namespace amlip {
namespace types {

class AmlipIdDataType;

/*!
 * @brief This class represents the unique identifier of AML-IP nodes and entities.
 * It is composed of an alphanumerical name and a random numerical id.
 * @ingroup AMLIP
 */
class AmlipId
{
public:

    /////
    // CONSTRUCTORS

    /*!
     * @brief New unique Id constructor.
     *
     * Create a new random unique Id.
     */
    AmlipId(
            const std::string& name = UNDEFINED_NAME_);

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object AmlipId that will be copied.
     * @note A pointer is created pointing to a newly created \c AmlipIdDataType object
     */
    AmlipId(
            const AmlipId& x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object AmlipId that will be copied.
     */
    AmlipId(
            AmlipId&& x);

    /*!
     * @brief Default destructor.
     */
    ~AmlipId();

    /////
    // OPERATORS

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object AmlipId that will be copied.
     */
    AmlipId& operator =(
            const AmlipId& x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object AmlipId that will be copied.
     */
    AmlipId& operator =(
            AmlipId&& x);

    /*!
     * @brief Comparison operator.
     * @param x AmlipId object to compare.
     */
    bool operator ==(
            const AmlipId& x) const;

    /*!
     * @brief Comparison operator.
     * @param x AmlipId object to compare.
     */
    bool operator !=(
            const AmlipId& x) const;

    /*!
     * @brief Comparison operator.
     * @param x AmlipId object to compare.
     */
    bool operator <(
            const AmlipId& x) const;

    /////
    // PUBLIC METHODS

    /*!
     * @brief This function gets the value of member \c name_ from object pointed to by \c data_
     */
    std::string name() const;

    /*!
     * @brief This function creates a string that uniquely describes this object.
     *
     * @note This string is forced to be valid as a DDS name for entity or topic.
     */
    std::string to_dds_string() const;

    /*!
     * @brief This function gets the value of attribute \c data_
     * @return Value of attribute \c data_ (a pointer, no the value pointed to)
     */
    std::shared_ptr<AmlipIdDataType> data() const;

    /**
     * @brief Return a DDS Data object copy from the internal data.
     *
     * @return AmlipIdDataType copy of the internal data.
     */
    AmlipIdDataType dds_data() const;

    /*!
     * @brief This function returns an \c AmlipId object constructed with the given name
     * @param name Value given as string to use as alphanumerical identifier
     */
    static AmlipId new_unique_id(
            const std::string& name);

protected:

    std::shared_ptr<AmlipIdDataType> data_;

    const static std::string UNDEFINED_NAME_;
};

//! \c AmlipId to stream serializator
std::ostream& operator <<(
        std::ostream& os,
        const AmlipId& id);

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

#endif // AMLIP_AMLIPCPP_TYPES_AMLIPID_HPP
