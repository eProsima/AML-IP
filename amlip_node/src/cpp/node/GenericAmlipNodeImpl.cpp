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
 * @file GenericAmlipNodeImpl.cpp
 */

#include <network/topics.hpp>
#include <node/GenericAmlipNodeImpl.hpp>

namespace eprosima {
namespace amlip {
namespace node {

GenericAmlipNodeImpl::GenericAmlipNodeImpl(
        const std::function<void(types::GenericType)> callback)
    : AmlipNodeImpl()
    , writer_(nullptr)
    , reader_(nullptr)
    , callback_(callback)
    , stop_(true)
{
    // Write current status
    write_status_(types::StatusKind::RUNNING);

    // Create reader from this participant
    eprosima::fastdds::dds::DataReaderQos reader_qos = eprosima::fastdds::dds::DATAREADER_QOS_DEFAULT;
    reader_qos.endpoint().history_memory_policy =
                eprosima::fastrtps::rtps::MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

    reader_ = participant_->create_reader<types::GenericType>(network::GENERIC_TOPIC, reader_qos);

    // Create writer from this participant
    eprosima::fastdds::dds::DataWriterQos writer_qos = eprosima::fastdds::dds::DATAWRITER_QOS_DEFAULT;
    writer_qos.endpoint().history_memory_policy =
                eprosima::fastrtps::rtps::MemoryManagementPolicy_t::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

    writer_ = participant_->create_writer<types::GenericType>(network::GENERIC_TOPIC, writer_qos);
}

GenericAmlipNodeImpl::~GenericAmlipNodeImpl()
{
    // Notify end of execution
    write_status_(types::StatusKind::DISABLED);

    // TODO: destroy reader correctly
    reader_.reset();

    // TODO: destroy writer correctly
    writer_.reset();
}

void GenericAmlipNodeImpl::publish(types::GenericType &data)
{
    std::cout << "DEBUG: Sending data" << std::endl;

    // Send data
    writer_->write(data);
}

bool GenericAmlipNodeImpl::wait_writer_matched()
{
    writer_->wait_writer_matched();

    return !writer_->stopped();
}

std::shared_ptr<types::GenericType> GenericAmlipNodeImpl::receive()
{
    std::cout << "DEBUG: Waiting for data to arrive" << std::endl;

    // Wait for new data
    reader_->wait_data_available();

    std::shared_ptr<types::GenericType> received_data = std::make_shared<types::GenericType>();

    // Check there is data available or it has been awake for stop (or any other future reason)
    if (reader_->is_data_available())
    {
        received_data = reader_->read();
    }

    return received_data;
}

void GenericAmlipNodeImpl::spin()
{
    std::cout << "DEBUG: spinning cpp" << std::endl;

    stop_.store(false);
    while (!stop_)
    {
        // Wait for new data
        reader_->wait_data_available();

        // Check there is data available or it has been awake for stop (or any other future reason)
        if (!reader_->is_data_available())
        {
            continue;
        }

        // New data available, take it and call callback
        std::shared_ptr<types::GenericType> data = reader_->read();
        callback_(*data);
    }
}

void GenericAmlipNodeImpl::stop()
{
    // Stop this entity
    stop_.store(true);

    // Stop Writer
    writer_->stop();

    // Stop Reader
    reader_->stop();
}

types::NodeKind GenericAmlipNodeImpl::node_kind_() const noexcept
{
    return types::NodeKind::GENERIC;
}

} /* namespace node */
} /* namespace amlip */
} /* namespace eprosima */
