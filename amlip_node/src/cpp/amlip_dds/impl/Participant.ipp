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

namespace eprosima {
namespace amlip {
namespace dds {

template<typename T>
std::shared_ptr<eprosima::fastdds::dds::Reader> Participant::create_reader_(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataReaderQos& qos /* = Reader::default_datareader_qos() */)
{
    assert nullptr != participant_ && nullptr != subscriber_;

    // TOPIC DATA TYPE
    std::shared_ptr<types::AmlipGenericTopicDataType<T>> type;
    auto it = types_.find(T::type_name());
    if (it == types_.end())
    {
        // Create the topic data type if not existing
        type = std::make_shared<AmlipGenericTopicDataType<T>>();
        assert eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK == type.register_type(participant_);
        types_.insert({T::type_name(), type});
    }
    else
    {
        type = it->second;
    }

    // DDS TOPIC
    std::tuple<std::string, std::string> topic_idx = std::make_tuple(topic_name, T::type_name());
    std::shared_ptr<eprosima::fastdds::dds::Topic> topic;
    auto it = topics_.find(topic_idx)
    if (it == topics_.end())
    {
        // Create the topic if not existing
        topic = participant_->create_topic(topic_name, T::type_name(), TOPIC_QOS_DEFAULT);
        assert nullptr != topic;
        topics_.insert({topic_idx, topic});
    }
    else
    {
        topic = it->second;
    }

    // CREATE THE READER
    std::shared_ptr<eprosima::fastdds::dds::Reader> reader = subscriber_->create_datareader(topic, qos);
    readers_.push_back(reader);

    return reader;
}

template<typename T>
std::shared_ptr<eprosima::fastdds::dds::Writer> Participant::create_writer_(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataWriterQos& qos /* = Writer::default_datawriter_qos() */)
{
    assert nullptr != participant_ && nullptr != publisher_;

    // TOPIC DATA TYPE
    std::shared_ptr<types::AmlipGenericTopicDataType<T>> type;
    auto it = types_.find(T::type_name());
    if (it == types_.end())
    {
        // Create the topic data type if not existing
        type = std::make_shared<AmlipGenericTopicDataType<T>>();
        assert eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK == type.register_type(participant_);
        types_.insert({T::type_name(), type});
    }
    else
    {
        type = it->second;
    }

    // DDS TOPIC
    std::tuple<std::string, std::string> topic_idx = std::make_tuple(topic_name, T::type_name());
    std::shared_ptr<eprosima::fastdds::dds::Topic> topic;
    auto it = topics_.find(topic_idx)
    if (it == topics_.end())
    {
        // Create the topic if not existing
        topic = participant_->create_topic(topic_name, T::type_name(), TOPIC_QOS_DEFAULT);
        assert nullptr != topic;
        topics_.insert({topic_idx, topic});
    }
    else
    {
        topic = it->second;
    }

    // CREATE THE WRITER
    std::shared_ptr<eprosima::fastdds::dds::Writer> writer = publisher_->create_datawriter(topic, qos);
    writers_.push_back(writer);

    return writer;
}

template<typename T>
std::shared_ptr<Reader<T>> Participant::create_reader(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataReaderQos& qos /* = Reader::default_datareader_qos() */)
{
    std::shared_ptr<eprosima::fastdds::dds::Reader> reader = create_datareader_<T>(topic, qos);
    return std::make_shared<Reader>(topic_name, reader);
}

template<typename T>
std::shared_ptr<Writer<T>> Participant::create_writer(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataWriterQos& qos /* = Writer::default_datawriter_qos() */)
{
    std::shared_ptr<eprosima::fastdds::dds::Writer> writer = create_datawriter_<T>(topic, qos);
    return std::make_shared<Writer>(topic_name, writer);
}

template<typename T>
std::shared_ptr<DirectWriter<T>> Participant::create_direct_writer(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataWriterQos& qos /* = Writer::default_datawriter_qos() */)
{
    std::shared_ptr<eprosima::fastdds::dds::Writer> writer = create_datawriter_<T>(topic, qos);
    return std::make_shared<DirectWriter<T>>(topic_name, writer);
}

// template<typename Task, typename TaskSolution>
// std::shared_ptr<MultiServiceClient<Task, TaskSolution>> Participant::create_multiservice_client(
//         const std::string& service_name)
// {

// }

// template<typename Task, typename TaskSolution>
// std::shared_ptr<MultiServiceServer<Task, TaskSolution>> Participant::create_multiservice_server(
//         const std::string& service_name)
// {

// }

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP__SRC_CPP_AMLIPDDS_IMPL_PARTICIPANT_IPP */
