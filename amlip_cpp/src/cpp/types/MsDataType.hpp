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
 * @file MsDataType.hpp
 */

#ifndef AMLIPCPP_TYPES_MSDATATYPE_HPP
#define AMLIPCPP_TYPES_MSDATATYPE_HPP

#include <types/MsReferenceDataType.hpp>

namespace eprosima {
namespace fastcdr {
// Forward declaration of the CDR class
class Cdr;
} // namespace fastcdr
} // namespace eprosima

namespace eprosima {
namespace amlip {
namespace types {

/**
 * TODO
 */
template <typename T>
class MsDataType : public MsReferenceDataType
{

    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, T);

public:

    MsDataType();

    MsDataType(
            const AmlipIdDataType& client_id,
            const TaskId& task_id,
            const AmlipIdDataType& server_id,
            const T& data);

    MsDataType(
            const MsReferenceDataType& reference,
            const T& data);

    MsDataType(
            MsReferenceDataType&& reference,
            T&& data);

    /*!
     * @brief This function serializes an object using CDR serialization.
     *
     * @param cdr CDR serialization object.
     *
     * @warning this method must be overriden in child class.
     */
    virtual void serialize(
            eprosima::fastcdr::Cdr& cdr) const override;

    /*!
     * @brief This function deserializes an object using CDR serialization.
     *
     * @param cdr CDR serialization object.
     *
     * @warning this method must be overriden in child class.
     */
    virtual void deserialize(
            eprosima::fastcdr::Cdr& cdr) override;

    /*!
     * @brief This function serializes the key members of an object using CDR serialization.
     *
     * @param cdr CDR serialization object.
     *
     * @warning this method must be overriden in child class.
     */
    virtual void serialize_key(
            eprosima::fastcdr::Cdr& cdr) const override;

    /*!
     * @brief This function returns the maximum serialized size of an object
     * depending on the buffer alignment.
     *
     * @param current_alignment Buffer alignment.
     *
     * @return Maximum serialized size.
     *
     * @warning this method must be overriden in child class.
     */
    static size_t get_max_cdr_serialized_size(
            size_t current_alignment = 0);

    /*!
     * @brief This function returns the serialized size of a data depending on the buffer alignment.

     * @param data Data which is calculated its serialized size.
     * @param current_alignment Buffer alignment.
     *
     * @return Serialized size.
     *
     * @warning this method must be overriden in child class.
     */
    static size_t get_cdr_serialized_size(
            const MsDataType& data,
            size_t current_alignment = 0);

    /*!
     * @brief This function returns the maximum serialized size of the Key of an object
     * depending on the buffer alignment.
     *
     * @param current_alignment Buffer alignment.
     *
     * @return Maximum serialized size.
     *
     * @warning this method must be overriden in child class.
     */
    static size_t get_key_max_cdr_serialized_size(
            size_t current_alignment = 0);

    /*!
     * @brief This function tells you if the Key has been defined for this type
     *
     * @warning this method must be overriden in child class.
     */
    static bool is_key_defined();

    /**
     * @brief Whether the type is bounded
     *
     * @warning this method must be overriden in child class.
     */
    static bool is_bounded();

    /**
     * @brief Whether the type is plain
     *
     * @warning this method must be overriden in child class.
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
     *
     * @warning this method must be overriden in child class.
     */
    static bool construct_sample(
            void* memory);

    /**
     * @brief Name of the Data Type. This name will be used as the DDS type name.
     *
     * @warning this method must be overriden in child class.
     */
    static std::string type_name();

    const T& data() const;

    void data(
            T new_value);

protected:

    static const char* DATA_TYPE_PREFIX_NAME_;

    T data_;
};

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <types/impl/MsDataType.ipp>

#endif // AMLIPCPP_TYPES_MSDATATYPE_HPP
