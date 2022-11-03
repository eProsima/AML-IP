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
 * @file AmlipGenericTopicDataType.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_TYPES_AMLIPGENERICTOPICDATATYPE_HPP
#define AMLIPCPP__SRC_CPP_TYPES_AMLIPGENERICTOPICDATATYPE_HPP

#include <fastdds/dds/topic/TopicDataType.hpp>
#include <fastrtps/utils/md5.h>

#include <cpp_utils/macros/macros.hpp>

#include <types/InterfaceDataType.hpp>

namespace eprosima {
namespace amlip {
namespace types {

//! Generic class that implements the parent of every AmlipGenericTopicDataType specialization class
class IBaseAmlipGenericTopicDataType
{
public:

    //! This virtual destructor is required so objects could be destroyed from its common interface.
    virtual ~IBaseAmlipGenericTopicDataType()
    {
    }

};

/*!
 * @brief This class represents the Generic TopicDataType of every AML-IP data type
 * @ingroup AMLIP
 *
 * Fast DDS requires a \c TopicDataType ("PubSubType") to be defined for every data type.
 * Instead of implementing a TopicDataType for every data type, this class implements a generic one that works
 * for every Data Type that inherits from \c InterfaceDataType class.
 *
 * @tparam T Type of the data that will be serialized/deserialized. It must inherit from InterfaceDataType.
 */
template <class T>
class AmlipGenericTopicDataType : public eprosima::fastdds::dds::TopicDataType, public IBaseAmlipGenericTopicDataType
{

    // Force T to be subclass of InterfaceDataType
    FORCE_TEMPLATE_SUBCLASS(InterfaceDataType, T);

public:

    //! Default constructor
    AmlipGenericTopicDataType();

    //! Default destructor
    virtual ~AmlipGenericTopicDataType();

    //! \c serialize method overriden from \c TopicDataType
    virtual bool serialize(
            void* data,
            eprosima::fastrtps::rtps::SerializedPayload_t* payload) override;

    //! \c deserialize method overriden from \c TopicDataType
    virtual bool deserialize(
            eprosima::fastrtps::rtps::SerializedPayload_t* payload,
            void* data) override;

    //! \c getSerializedSizeProvider method overriden from \c TopicDataType
    virtual std::function<uint32_t()> getSerializedSizeProvider(
            void* data) override;

    //! \c getKey method overriden from \c TopicDataType
    virtual bool getKey(
            void* data,
            eprosima::fastrtps::rtps::InstanceHandle_t* ihandle,
            bool force_md5 = false) override;

    //! \c createData method overriden from \c TopicDataType
    virtual void* createData() override;

    //! \c deleteData method overriden from \c TopicDataType
    virtual void deleteData(
            void* data) override;

    //! \c is_bounded method overriden from \c TopicDataType
    virtual bool is_bounded() const override;

    //! \c is_plain method overriden from \c TopicDataType
    virtual bool is_plain() const override;

    //! \c construct_sample method overriden from \c TopicDataType
    virtual bool construct_sample(
            void* memory) const override;

protected:

    //! MD5 variable
    MD5 md5_;

    //! Buffer to store key
    unsigned char* key_buffer_;
};

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <types/impl/AmlipGenericTopicDataType.ipp>

#endif // AMLIPCPP__SRC_CPP_TYPES_AMLIPGENERICTOPICDATATYPE_HPP
