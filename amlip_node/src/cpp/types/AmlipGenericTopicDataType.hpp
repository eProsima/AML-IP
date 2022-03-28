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
 * @file AmlipGenericTopicDataType.hpp
 */

#ifndef AMLIP__SRC_CPP_AMLIPTYPES_AMLIPGENERICTOPICDATATYPE_HPP
#define AMLIP__SRC_CPP_AMLIPTYPES_AMLIPGENERICTOPICDATATYPE_HPP

#include <fastdds/dds/topic/TopicDataType.hpp>
#include <fastrtps/utils/md5.h>

#include <ddsrouter_utils/macros.hpp>

#include <amlip_node/types/InterfaceDataType.hpp>

namespace eprosima {
namespace amlip {
namespace types {

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
 */
template <class T>
class AmlipGenericTopicDataType : public eprosima::fastdds::dds::TopicDataType, public IBaseAmlipGenericTopicDataType
{

    // Force T to be subclass of InterfaceDataType
    FORCE_TEMPLATE_SUBCLASS(InterfaceDataType, T);

public:

    AmlipGenericTopicDataType();

    virtual ~AmlipGenericTopicDataType();

    virtual bool serialize(
            void* data,
            eprosima::fastrtps::rtps::SerializedPayload_t* payload) override;

    virtual bool deserialize(
            eprosima::fastrtps::rtps::SerializedPayload_t* payload,
            void* data) override;

    virtual std::function<uint32_t()> getSerializedSizeProvider(
            void* data) override;

    virtual bool getKey(
            void* data,
            eprosima::fastrtps::rtps::InstanceHandle_t* ihandle,
            bool force_md5 = false) override;

    virtual void* createData() override;

    virtual void deleteData(
            void* data) override;

    // virtual bool is_bounded() const override;

    // virtual bool is_plain() const override;

    // virtual bool construct_sample(
    //         void* memory) const override;

    MD5 m_md5;
    unsigned char* m_keyBuffer;
};

} /* namespace types */
} /* namespace amlip */
} /* namespace eprosima */

// Include implementation template file
#include <types/impl/AmlipGenericTopicDataType.ipp>

#endif // AMLIP__SRC_CPP_AMLIPTYPES_AMLIPGENERICTOPICDATATYPE_HPP
