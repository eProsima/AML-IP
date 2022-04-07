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

/**
 * @file Participant.ipp
 */

#ifndef AMLIPCPP__SRC_CPP_DDS_IMPL_PARTICIPANT_IPP
#define AMLIPCPP__SRC_CPP_DDS_IMPL_PARTICIPANT_IPP

#include <assert.h>

#include <ddsrouter_utils/exception/InitializationException.hpp>
#include <ddsrouter_utils/exception/InconsistencyException.hpp>

#include <types/InterfaceDataType.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

template<typename T>
eprosima::fastrtps::types::ReturnCode_t Participant::register_type_() noexcept
{
    // Force T to be subclass of InterfaceDataType
    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, T);

    auto it_types = types_.find(T::type_name());
    if (it_types == types_.end())
    {
        // Create type support if not existing and register participant
        eprosima::fastdds::dds::TypeSupport type_support(new types::AmlipGenericTopicDataType<T>());

        eprosima::fastrtps::types::ReturnCode_t ret = type_support.register_type(participant_.get());

        if (ret)
        {
            // Add Type to map
            types_.insert({T::type_name(), type_support});
        }
        else
        {
            return ret;
        }
    }
    else
    {
        return eprosima::fastrtps::types::ReturnCode_t::RETCODE_PRECONDITION_NOT_MET;
    }
}

template<typename T>
std::shared_ptr<eprosima::fastdds::dds::Topic> Participant::get_topic_(const std::string& topic_name)
{
    // Force T to be subclass of InterfaceDataType
    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, T);

    // Check if it exists
    std::pair<std::string, std::string> topic_idx = std::make_pair(topic_name, T::type_name());
    auto it_topics = topics_.find(topic_idx);
    if (it_topics != topics_.end())
    {
        return it_topics->second;
    }

    // Check if topic and type are coherent
    for (auto& topic_it : topics_)
    {
        if (topic_it.second->get_type_name() == topic_name)
        {
            // The Topic already exists with other type
            throw ddsrouter::utils::InconsistencyException(
                STR_ENTRY << "Topic " << topic_name << " already exists with other type that is not "
                          << T::type_name() << ".");
        }
    }

    // Register type (if already registered nothing happens)
    register_type_<T>();

    // Create topic
    std::shared_ptr<eprosima::fastdds::dds::Topic> topic(
        participant_->create_topic(
            topic_name,
            T::type_name(),
            eprosima::fastdds::dds::TOPIC_QOS_DEFAULT),
        [this](eprosima::fastdds::dds::Topic* topic)
        {
            // deleter for shared ptr
            this->participant_->delete_topic(topic);
        }
    );
    if (nullptr == topic)
    {
        throw ddsrouter::utils::InitializationException(
            STR_ENTRY << "Failed to create topic " << topic_name << ".");
    }

    // Add new topic to map
    topics_[topic_idx] = topic;

    return topic;
}

// template<typename T>
// std::shared_ptr<eprosima::fastdds::dds::DataReader> Participant::create_datareader_(
//         const std::string& topic_name,
//         const eprosima::fastdds::dds::DataReaderQos& qos /* = Reader::default_datareader_qos() */)
// {
//     assert(nullptr != participant_ && nullptr != subscriber_);

//     // Get or create topic
//     std::shared_ptr<eprosima::fastdds::dds::Topic> topic = register_topic<T>(topic_name);

//     // Create the datareader
//     std::shared_ptr<eprosima::fastdds::dds::DataReader> datareader(
//             subscriber_->create_datareader(topic.get(), qos),
//             [this](eprosima::fastdds::dds::DataReader* datareader)
//             {
//                 // deleter for shared ptr
//                 this->subscriber_->delete_datareader(datareader);
//             }
//         );

//     // TODO throw ddsrouter::utils::exception if nullptr
//     assert(nullptr != datareader);

//     return datareader;
// }

// template<typename T>
// std::shared_ptr<eprosima::fastdds::dds::DataWriter> Participant::create_datawriter_(
//         const std::string& topic_name,
//         const eprosima::fastdds::dds::DataWriterQos& qos /* = Writer::default_datawriter_qos() */)
// {
//     assert(nullptr != participant_ && nullptr != publisher_);

//     // Get or create topic
//     std::shared_ptr<eprosima::fastdds::dds::Topic> topic = register_topic<T>(topic_name);

//     // Create the datawriter
//     std::shared_ptr<eprosima::fastdds::dds::DataWriter> datawriter(
//             publisher_->create_datawriter(topic.get(), qos),
//             [this](eprosima::fastdds::dds::DataWriter* datawriter)
//             {
//                 // deleter for shared ptr
//                 this->publisher_->delete_datawriter(datawriter);
//             }
//         );

//     // TODO throw ddsrouter::utils::exception if nullptr
//     assert(nullptr != datawriter);

//     return datawriter;
// }

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_DDS_IMPL_PARTICIPANT_IPP */