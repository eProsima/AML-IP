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

#ifndef AMLIP__SRC_CPP_AMLIPDDS_IMPL_PARTICIPANT_IPP
#define AMLIP__SRC_CPP_AMLIPDDS_IMPL_PARTICIPANT_IPP

#include <assert.h>

namespace eprosima {
namespace amlip {
namespace dds {

template<typename T>
eprosima::fastdds::dds::TypeSupport Participant::register_type() noexcept
{
    auto it_types = types_.find(T::type_name());
    if (it_types == types_.end())
    {
        // Create type support if not existing and register participant
        eprosima::fastdds::dds::TypeSupport type_support;
        type_support.reset(new types::AmlipGenericTopicDataType<T>());
        eprosima::fastrtps::types::ReturnCode_t ret = type_support.register_type(participant_.get());
        // TODO convert this into an exception
        assert(eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK == ret);
        types_.insert({T::type_name(), type_support});

        return type_support;
    }
    else
    {
        // Type already registered
        return it_types->second;
    }
}

template<typename T>
std::shared_ptr<eprosima::fastdds::dds::Topic> Participant::register_topic(const std::string& topic_name)
{
    // Check if it exists
    std::pair<std::string, std::string> topic_idx = std::make_pair(topic_name, T::type_name());
    auto it_topics = topics_.find(topic_idx);
    if (it_topics != topics_.end())
    {
        return it_topics->second;
    }

    // TODO check if topic and type are coherent

    // Register type (if already registered nothing happens)
    register_type<T>();

    // Create topic if not existing
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

    // Add new topic to map
    topics_[topic_idx] = topic;

    return topic;
}

template<typename T>
std::shared_ptr<eprosima::fastdds::dds::DataReader> Participant::create_datareader_(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataReaderQos& qos /* = Reader::default_datareader_qos() */)
{
    assert(nullptr != participant_ && nullptr != subscriber_);

    // Get or create topic
    std::shared_ptr<eprosima::fastdds::dds::Topic> topic = register_topic<T>(topic_name);

    // Create the datareader
    std::shared_ptr<eprosima::fastdds::dds::DataReader> datareader(
            subscriber_->create_datareader(topic.get(), qos),
            [this](eprosima::fastdds::dds::DataReader* datareader)
            {
                // deleter for shared ptr
                this->subscriber_->delete_datareader(datareader);
            }
        );

    // TODO throw exception if nullptr
    assert(nullptr != datareader);

    return datareader;
}

template<typename T>
std::shared_ptr<eprosima::fastdds::dds::DataWriter> Participant::create_datawriter_(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataWriterQos& qos /* = Writer::default_datawriter_qos() */)
{
    assert(nullptr != participant_ && nullptr != publisher_);

    // Get or create topic
    std::shared_ptr<eprosima::fastdds::dds::Topic> topic = register_topic<T>(topic_name);

    // Create the datawriter
    std::shared_ptr<eprosima::fastdds::dds::DataWriter> datawriter(
            publisher_->create_datawriter(topic.get(), qos),
            [this](eprosima::fastdds::dds::DataWriter* datawriter)
            {
                // deleter for shared ptr
                this->publisher_->delete_datawriter(datawriter);
            }
        );

    // TODO throw exception if nullptr
    assert(nullptr != datawriter);

    return datawriter;
}

template<typename T>
std::shared_ptr<Reader<T>> Participant::create_reader(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataReaderQos& qos /* = Reader::default_datareader_qos() */)
{
    std::shared_ptr<eprosima::fastdds::dds::DataReader> data_reader = create_datareader_<T>(topic_name, qos);
    std::shared_ptr<Reader<T>> reader = std::make_shared<Reader<T>>(topic_name, data_reader);
    data_reader->set_listener(reader.get());
    return reader;
}

template<typename T>
std::shared_ptr<Writer<T>> Participant::create_writer(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataWriterQos& qos /* = Writer::default_datawriter_qos() */)
{
    std::shared_ptr<eprosima::fastdds::dds::DataWriter> data_writer = create_datawriter_<T>(topic_name, qos);
    std::shared_ptr<Writer<T>> writer = std::make_shared<Writer<T>>(topic_name, data_writer);
    data_writer->set_listener(writer.get());
    return writer;
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP__SRC_CPP_AMLIPDDS_IMPL_PARTICIPANT_IPP */
