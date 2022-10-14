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
 * @file targetedreader.cpp
 *
 */

#include <cpp_utils/Log.hpp>

#include <types/AmlipIdDataType.hpp>
#include <dds/Participant.hpp>

int main(
        int argc,
        char** argv)
{
    // Activate log
    eprosima::utils::Log::SetVerbosity(eprosima::utils::Log::Kind::Info);

    logUser(AMLIP_MANUAL_TEST, "Starting Manual Test TargetedReader execution. Creating Participant...");

    {
        // Create Participant
        eprosima::amlip::dds::Participant participant("ManualTestParticipant");

        logUser(AMLIP_MANUAL_TEST, "Created Participant: " << participant << ". Creating Writer to send Id...");

        // Create Writer to send id (it must be transient local and reliable)
        eprosima::fastdds::dds::DataWriterQos qos;
        qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
        qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
        qos.history().kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_ALL_HISTORY_QOS;

        std::shared_ptr<eprosima::amlip::dds::Writer<eprosima::amlip::types::AmlipIdDataType>> writer =
                participant.create_writer<eprosima::amlip::types::AmlipIdDataType>("manual_test_topic", qos);

        logUser(AMLIP_MANUAL_TEST, "Created Writer. Sending Id...");

        // Send own id
        eprosima::amlip::types::AmlipIdDataType own_id = participant.id();
        writer->publish(own_id);

        logUser(AMLIP_MANUAL_TEST, "Writer sent data: " << own_id << ". Creating Targeted Reader...");

        // Create Targeted reader
        std::shared_ptr<eprosima::amlip::dds::TargetedReader<eprosima::amlip::types::AmlipIdDataType>> reader =
                participant.create_targeted_reader<eprosima::amlip::types::AmlipIdDataType>("manual_test_topic");

        logUser(AMLIP_MANUAL_TEST, "Created Targeted Reader. Waiting data...");

        // Wait for data
        reader->wait_data_available();

        logUser(AMLIP_MANUAL_TEST, "Data received. Reading data...");

        // Read data
        eprosima::amlip::types::AmlipIdDataType received_data = reader->read();

        logUser(AMLIP_MANUAL_TEST, "Read message: " << received_data << ". Destroying entities...");
    }

    logUser(AMLIP_MANUAL_TEST, "Finishing Manual Test Reader execution.");

    return 0;
}
