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
 * @file DdsHandler.ipp
 */

#ifndef AMLIPCPP__SRC_CPP_DDS_IMPL_DDSHANDLER_IPP
#define AMLIPCPP__SRC_CPP_DDS_IMPL_DDSHANDLER_IPP

#include <cpp_utils/exception/InitializationException.hpp>
#include <cpp_utils/exception/InconsistencyException.hpp>
#include <cpp_utils/Log.hpp>
#include <cpp_utils/macros/macros.hpp>

#include <amlip_cpp/types/InterfaceDataType.hpp>
#include <dds/network_utils/topic.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

template<typename T>
eprosima::fastrtps::types::ReturnCode_t DdsHandler::register_type_() noexcept
{
    // Force T to be subclass of InterfaceDataType
    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, T);

    auto it_types = types_.find(T::type_name());
    if (it_types == types_.end())
    {
        // Create type support if not existing and register DdsHandler
        eprosima::fastdds::dds::TypeSupport type_support(new types::AmlipGenericTopicDataType<T>());

        // Deactivate dynamic types
        type_support->auto_fill_type_information(false);
        type_support->auto_fill_type_object(false);

        // TODO: make mangling more independent from code
        eprosima::fastrtps::types::ReturnCode_t ret = participant_->register_type(
            type_support,
            utils::type_name_mangling(T::type_name()));

        if (!ret)
        {
            return ret;
        }
        else
        {
            // Add Type to map
            types_.insert({T::type_name(), type_support});

            logInfo(AMLIPCPP_DDSHANDLER, "Registered Type " << T::type_name() << ".");
            logDebug(AMLIPCPP_DDSHANDLER, "Registered Type " << T::type_name() <<
                    " with class: " << TYPE_NAME(T) << ".");

            return ret;
        }
    }
    else
    {
        return eprosima::fastrtps::types::ReturnCode_t::RETCODE_PRECONDITION_NOT_MET;
    }
}

template<typename T>
eprosima::utils::LesseePtr<eprosima::fastdds::dds::Topic> DdsHandler::get_topic_(
        const std::string& topic_name)
{
    // Force T to be subclass of InterfaceDataType
    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, T);

    // Check if it exists
    std::pair<std::string, std::string> topic_idx = std::make_pair(topic_name, T::type_name());
    auto it_topics = topics_.find(topic_idx);
    if (it_topics != topics_.end())
    {
        return it_topics->second.lease();
    }

    // Check if topic and type are coherent
    for (auto& topic_it : topics_)
    {
        // Check if this topic name already exist (this case could only happen with different type names)
        if (topic_it.first.first == topic_name)
        {
            // The Topic already exists with other type
            throw eprosima::utils::InconsistencyException(
                      STR_ENTRY << "Topic " << topic_name << " already exists with other type that is not "
                                << T::type_name() << ".");
        }
    }

    // Register type (if already registered nothing happens)
    eprosima::fastrtps::types::ReturnCode_t ret = register_type_<T>();

    if (ret != eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK &&
            ret != eprosima::fastrtps::types::ReturnCode_t::RETCODE_PRECONDITION_NOT_MET)
    {
        throw eprosima::utils::InitializationException(
                  STR_ENTRY << "Topic " << topic_name << " creation failed due to type registration.");
    }

    // Create topic
    // TODO: make mangling more independent from code
    eprosima::utils::OwnerPtr<eprosima::fastdds::dds::Topic> topic(
        participant_->create_topic(
            utils::topic_name_mangling(topic_name),
            utils::type_name_mangling(T::type_name()),
            eprosima::fastdds::dds::TOPIC_QOS_DEFAULT),
        [this](eprosima::fastdds::dds::Topic* topic)
        {
            logDebug(AMLIPCPP_DDSHANDLER, "AutoDeleting Topic " << topic->get_name() << ".");

            // deleter for shared ptr
            logDebug(AMLIPCPP_DDSHANDLER, "Destroying topic " << topic->get_name() << ".");
            this->participant_->delete_topic(topic);
        }
        );
    if (nullptr == topic)
    {
        throw eprosima::utils::InitializationException(
                  STR_ENTRY << "Failed to create topic " << topic_name << ".");
    }

    // Add new topic to map
    auto topic_lessee = topic.lease();
    topics_[topic_idx] = std::move(topic);

    logInfo(AMLIPCPP_DDSHANDLER, "Registered Topic " << topic_name << " with type: " << T::type_name() << ".");

    return topic_lessee;
}

template <typename T>
eprosima::utils::LesseePtr<eprosima::fastdds::dds::DataWriter> DdsHandler::create_datawriter(
        const std::string topic_name,
        eprosima::fastdds::dds::DataWriterQos qos,
        eprosima::fastdds::dds::DataWriterListener* listener /* = nullptr */)
{
    // Force T to be subclass of InterfaceDataType
    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, T);

    logDebug(AMLIPCPP_DDSHANDLER, "Creating DataWriter in topic " << topic_name << ".");

    // Get Topic (in case it does already exist return reference)
    eprosima::utils::LesseePtr<eprosima::fastdds::dds::Topic> topic_ =
            get_topic_<T>(topic_name);

    // Lock the Topic so its pointer is used to create datawriter.
    // This variable will be destroyed at the end of the function and will release mutex
    auto topic_locked_ptr = topic_.lock();
    if (!topic_locked_ptr)
    {
        throw eprosima::utils::InitializationException(
                  STR_ENTRY << "Failed to create DataWriter " << topic_name << " after Participant destruction.");
    }

    // Create DataWriter
    eprosima::utils::OwnerPtr<eprosima::fastdds::dds::DataWriter> datawriter(
        publisher_->create_datawriter(
            topic_locked_ptr.get(),
            qos,
            listener),
        [this](eprosima::fastdds::dds::DataWriter* datawriter)
        {
            logDebug(AMLIPCPP_DDSHANDLER, "AutoDestroying DataWriter " << datawriter->guid() <<
                " in topic " << datawriter->get_topic()->get_name() << ".");

            // deleter for shared ptr
            this->publisher_->delete_datawriter(datawriter);
        }
        );
    if (nullptr == datawriter)
    {
        throw eprosima::utils::InitializationException(
                  STR_ENTRY << "Failed to create DataWriter " << topic_name << ".");
    }

    logInfo(AMLIPCPP_DDSHANDLER, "DataWriter created in topic " << topic_name <<
            " with GUID: " << datawriter->guid() << ".");

    // Store datawriter
    auto datawriter_lessee = datawriter.lease();
    datawriters_.push_back(std::move(datawriter));

    return datawriter_lessee;

}

template <typename T>
eprosima::utils::LesseePtr<eprosima::fastdds::dds::DataReader> DdsHandler::create_datareader(
        const std::string topic_name,
        eprosima::fastdds::dds::DataReaderQos qos,
        eprosima::fastdds::dds::DataReaderListener* listener /* = nullptr */)
{
    // Force T to be subclass of InterfaceDataType
    FORCE_TEMPLATE_SUBCLASS(types::InterfaceDataType, T);

    logDebug(AMLIPCPP_DDSHANDLER, "Creating DataReader in topic " << topic_name << ".");

    // Get Topic (in case it does already exist return reference)
    eprosima::utils::LesseePtr<eprosima::fastdds::dds::Topic> topic_ =
            get_topic_<T>(topic_name);

    // Lock the Topic so its pointer is used to create datareader.
    // This variable will be destroyed at the end of the function and will release mutex
    auto topic_locked_ptr = topic_.lock();
    if (!topic_locked_ptr)
    {
        throw eprosima::utils::InitializationException(
                  STR_ENTRY << "Failed to create DataReader " << topic_name << " after Participant destruction.");
    }

    // Create DataReader
    eprosima::utils::OwnerPtr<eprosima::fastdds::dds::DataReader> datareader(
        subscriber_->create_datareader(
            topic_locked_ptr.get(),
            qos,
            listener),
        [this](eprosima::fastdds::dds::DataReader* datareader)
        {
            logDebug(AMLIPCPP_DDSHANDLER, "AutoDestroying DataReader " << datareader->guid() <<
                " in topic " << datareader->get_topicdescription()->get_name() << ".");

            // deleter for shared ptr
            this->subscriber_->delete_datareader(datareader);
        }
        );
    if (nullptr == datareader)
    {
        throw eprosima::utils::InitializationException(
                  STR_ENTRY << "Failed to create DataReader " << topic_name << ".");
    }

    logInfo(AMLIPCPP_DDSHANDLER, "DataReader created in topic " << topic_name <<
            " with GUID: " << datareader->guid() << ".");

    // Stor datareader
    auto datareader_lessee = datareader.lease();
    datareaders_.push_back(std::move(datareader));

    return datareader_lessee;
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_DDS_IMPL_DDSHANDLER_IPP */
