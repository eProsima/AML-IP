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

#include <array>
#include <limits>
#include <memory>
#include <ostream>
#include <string>

#include <types/AmlipIdDataType.hpp>

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
    AmlipId(const std::string& name = UNDEFINED_NAME_);

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object AmlipId that will be copied.
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

    /////
    // PUBLIC METHODS

    /*!
     * @brief This function copies the value in member id
     * @param _id New value to be copied in member id
     */
    std::string name() const;

    std::shared_ptr<AmlipIdDataType> data() const;

    static AmlipId new_unique_id(const std::string& name);

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
