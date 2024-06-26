// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
 * @file ModelStatisticsDataType.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_TYPES_MODEL_MODELSTATISTICSDATATYPE_HPP
#define AMLIPCPP__SRC_CPP_TYPES_MODEL_MODELSTATISTICSDATATYPE_HPP

#include <atomic>
#include <string>
#include <vector>

#include <fastcdr/config.h>

#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <amlip_cpp/types/InterfaceDataType.hpp>


namespace eprosima {
namespace amlip {
namespace types {

using ByteType = uint8_t;

/*!
 * @brief This class represents the structure ModelStatisticsDataType, which implements \c InterfaceDataType and hence serves
 * as a communication interface to \c ModelStatistics class.
 * @ingroup AMLIP
 */
class ModelStatisticsDataType : public InterfaceDataType
{
public:

    /*!
     * @brief Default constructor.
     */
    AMLIP_CPP_DllAPI ModelStatisticsDataType();

    /*!
     * @brief Constructor with name.
     * @param name New value to be copied in member id \c name_
     */
    AMLIP_CPP_DllAPI ModelStatisticsDataType(
            const std::string& name);

    /*!
     * @brief Constructor with name given in char*.
     * @param name New value to be copied in member id \c name_
     */
    AMLIP_CPP_DllAPI ModelStatisticsDataType(
            const char* name);

    /*!
     * @brief Constructor with name, data given in void* and data size.
     * @param name New value to be copied in member id \c name_
     * @param data New value to be copied in member id \c data_
     * @param size New value to be copied in member id \c data_size_
     * @param take_ownership
     */
    AMLIP_CPP_DllAPI ModelStatisticsDataType(
            const std::string& name,
            void* data,
            const uint32_t size,
            bool take_ownership = true);

    /*!
     * @brief Constructor with name and data given in std::vector<uint8_t>.
     * @param name New value to be copied in member id \c name_
     * @param bytes New value to be copied in member id \c data_
     * @param take_ownership
     */
    AMLIP_CPP_DllAPI ModelStatisticsDataType(
            const std::string& name,
            const std::vector<ByteType>& bytes,
            bool take_ownership = true);

    /*!
     * @brief Constructor with name and data.
     * @param name New value to be copied in member id \c name_
     * @param bytes New value to be copied in member id \c data_
     * @param take_ownership
     */
    AMLIP_CPP_DllAPI ModelStatisticsDataType(
            const std::string& name,
            const std::string& bytes,
            bool take_ownership = true);

    /*!
     * @brief Default destructor.
     */
    AMLIP_CPP_DllAPI virtual ~ModelStatisticsDataType();

    /*!
     * @brief Copy constructor.
     * @param x Reference to the object ModelStatisticsDataType that will be copied.
     */
    AMLIP_CPP_DllAPI ModelStatisticsDataType(
            const ModelStatisticsDataType& x);

    /*!
     * @brief Move constructor.
     * @param x Reference to the object ModelStatisticsDataType that will be copied.
     */
    AMLIP_CPP_DllAPI ModelStatisticsDataType(
            ModelStatisticsDataType&& x);

    /*!
     * @brief Copy assignment.
     * @param x Reference to the object ModelStatisticsDataType that will be copied.
     */
    AMLIP_CPP_DllAPI ModelStatisticsDataType& operator =(
            const ModelStatisticsDataType& x);

    /*!
     * @brief Move assignment.
     * @param x Reference to the object ModelStatisticsDataType that will be copied.
     */
    AMLIP_CPP_DllAPI ModelStatisticsDataType& operator =(
            ModelStatisticsDataType&& x);

    /*!
     * @brief Comparison operator.
     * @param x ModelStatisticsDataType object to compare.
     */
    AMLIP_CPP_DllAPI bool operator ==(
            const ModelStatisticsDataType& x) const;

    /*!
     * @brief Comparison operator.
     * @param x ModelStatisticsDataType object to compare.
     */
    AMLIP_CPP_DllAPI bool operator !=(
            const ModelStatisticsDataType& x) const;

    /*!
     * @brief Comparison operator.
     * @param x ModelStatisticsDataType object to compare.
     */
    AMLIP_CPP_DllAPI bool operator <(
            const ModelStatisticsDataType& x) const;

    /*!
     * @brief This function gets the value of member \c name_ as string
     * @return Value of member \c name_ as string
     */
    AMLIP_CPP_DllAPI std::string name() const;

    /*!
     * @return Returns reference to attribute \c name_
     */
    AMLIP_CPP_DllAPI std::string& name();

    /*!
     * @brief This function copies the value in member \c name_
     * @param name New value to be copied in member id \c name_
     */
    AMLIP_CPP_DllAPI void name(
            const std::string& name);

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
     * @brief Returns value of attribute \c data_ in std::string
     */
    AMLIP_CPP_DllAPI std::string to_string() const noexcept;

    /*!
     * @brief Returns value of attribute \c data_ in std::vector<uint8_t>
     */
    AMLIP_CPP_DllAPI std::vector<ByteType> to_vector() const noexcept;

    /*!
     * @brief Returns value of attribute \c data_size_
     */
    AMLIP_CPP_DllAPI uint32_t data_size() const;

    /*!
     * @brief Returns reference to attribute \c data_size_
     */
    AMLIP_CPP_DllAPI uint32_t& data_size();

    /*!
     * @brief This function gets the value in member \c rand_id as array of octets
     * @return Value of member \c rand_id_ as array of octets
     */
    AMLIP_CPP_DllAPI AmlipIdDataType server_id() const;

    /*!
     * @return Returns reference to attribute \c server_id_
     */
    AMLIP_CPP_DllAPI AmlipIdDataType& server_id();

    /*!
     * @brief This function copies the value in member \c id_
     * @param id New value to be copied in member id \c id_
     */
    AMLIP_CPP_DllAPI void server_id(
            const AmlipIdDataType& id);

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

    AMLIP_CPP_DllAPI static uint32_t max_cdr_typesize_;

    static constexpr uint32_t max_key_cdr_typesize_ {0UL};

protected:

    std::string name_;

    void* data_;
    uint32_t data_size_;
    std::atomic<bool> has_been_allocated_;

    AmlipIdDataType server_id_;

    static const char* TYPE_NAME_;

};

//! \c ModelStatisticsDataType to stream serializator
std::ostream& operator <<(
        std::ostream& os,
        const ModelStatisticsDataType& id);

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

#endif // AMLIPCPP__SRC_CPP_TYPES_MODEL_MODELSTATISTICSDATATYPE_HPP
