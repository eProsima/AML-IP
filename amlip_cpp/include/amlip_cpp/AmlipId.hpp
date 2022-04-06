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
 * @file AmlipId.hpp
 */

#ifndef AMLIP_AMLIPCPP_AMLIPID_HPP
#define AMLIP_AMLIPCPP_AMLIPID_HPP

#include <array>
#include <limits>
#include <ostream>

namespace eprosima {
namespace amlip {
namespace types {

class AmlipIdDataType;

/*!
 * @brief This class represents the structure AmlipId defined by the user in the IDL file.
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
    AmlipId(const std::string& name = "aNodeHasNoName");

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

    static AmlipIdDataType new_unique_id();

protected:

    std::shared_ptr<AmlipIdDataType> data_;
};

//! \c AmlipId to stream serializator
std::ostream& operator <<(
        std::ostream& os,
        const AmlipId& id);

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

#endif // AMLIP_AMLIPCPP_AMLIPID_HPP
