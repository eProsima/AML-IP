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
eprosima::fastdds::dds::DataReader* Participant::create_datareader_(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataReaderQos& qos /* = Reader::default_datareader_qos() */)
{
    assert(nullptr != participant_ && nullptr != subscriber_);

    // TYPE SUPPORT
    auto it_types = types_.find(T::type_name());
    if (it_types == types_.end())
    {
        // Create type support if not existing and register participant
        eprosima::fastdds::dds::TypeSupport type_support;
        type_support.reset(new types::AmlipGenericTopicDataType<T>());
        eprosima::fastrtps::types::ReturnCode_t ret = type_support.register_type(participant_);
        assert(eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK == ret);
        types_.insert({T::type_name(), type_support});
    }

    // DDS TOPIC
    std::pair<std::string, std::string> topic_idx = std::make_pair(topic_name, T::type_name());
    eprosima::fastdds::dds::Topic* topic;
    auto it_topics = topics_.find(topic_idx);
    if (it_topics == topics_.end())
    {
        // Create the topic if not existing
        topic = participant_->create_topic(topic_name, T::type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
        assert(nullptr != topic);
        topics_.insert({topic_idx, topic});
    }
    else
    {
        topic = it_topics->second;
    }

    // CREATE THE READER
    eprosima::fastdds::dds::DataReader* datareader = subscriber_->create_datareader(topic, qos);
    assert(nullptr != datareader);
    datareaders_.push_back(datareader);

    return datareader;
}

template<typename T>
eprosima::fastdds::dds::DataWriter* Participant::create_datawriter_(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataWriterQos& qos /* = Writer::default_datawriter_qos() */)
{
    assert(nullptr != participant_ && nullptr != publisher_);

    // TYPE SUPPORT
    auto it_types = types_.find(T::type_name());
    if (it_types == types_.end())
    {
        // Create type support if not existing and register participant
        eprosima::fastdds::dds::TypeSupport type_support;
        type_support.reset(new types::AmlipGenericTopicDataType<T>());
        eprosima::fastrtps::types::ReturnCode_t ret = type_support.register_type(participant_);
        assert(eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK == ret);
        types_.insert({T::type_name(), type_support});
    }

    // DDS TOPIC
    std::pair<std::string, std::string> topic_idx = std::make_pair(topic_name, T::type_name());
    eprosima::fastdds::dds::Topic* topic;
    auto it_topics = topics_.find(topic_idx);
    if (it_topics == topics_.end())
    {
        // Create the topic if not existing
        topic = participant_->create_topic(topic_name, T::type_name(), eprosima::fastdds::dds::TOPIC_QOS_DEFAULT);
        assert(nullptr != topic);
        topics_.insert({topic_idx, topic});
    }
    else
    {
        topic = it_topics->second;
    }

    // CREATE THE WRITER
    eprosima::fastdds::dds::DataWriter* datawriter = publisher_->create_datawriter(topic, qos);
    assert(nullptr != datawriter);
    datawriters_.push_back(datawriter);

    return datawriter;
}

template<typename T>
std::shared_ptr<Reader<T>> Participant::create_reader(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataReaderQos& qos /* = Reader::default_datareader_qos() */)
{
    eprosima::fastdds::dds::DataReader* data_reader = create_datareader_<T>(topic_name, qos);
    std::shared_ptr<Reader<T>> reader = std::make_shared<Reader<T>>(topic_name, data_reader);
    data_reader->set_listener(reader.get());
    return reader;
}

template<typename T>
std::shared_ptr<Writer<T>> Participant::create_writer(
        const std::string& topic_name,
        const eprosima::fastdds::dds::DataWriterQos& qos /* = Writer::default_datawriter_qos() */)
{
    eprosima::fastdds::dds::DataWriter* data_writer = create_datawriter_<T>(topic_name, qos);
    std::shared_ptr<Writer<T>> writer = std::make_shared<Writer<T>>(topic_name, data_writer);
    data_writer->set_listener(writer.get());
    return writer;
}

// template<typename T>
// std::shared_ptr<DirectWriter<T>> Participant::create_direct_writer(
//         const std::string& topic_name,
//         const eprosima::fastdds::dds::DataWriterQos& qos /* = Writer::default_datawriter_qos() */)
// {
    // return std::make_shared<DirectWriter<T>>(topic_name, create_datawriter_<T>(topic, qos));
// }

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
