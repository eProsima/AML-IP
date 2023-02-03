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
 * @file directwriter.cpp
 *
 */

#include <thread>

#include <cpp_utils/Log.hpp>

#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <dds/Participant.hpp>

int main(
        int argc,
        char** argv)
{
    // Activate log
    eprosima::utils::Log::SetVerbosity(eprosima::utils::Log::Kind::Info);

    logUser(AMLIPCPP_MANUAL_TEST, "Starting Manual Test DirectWriter execution. Creating Participant...");

    {
        // Create Participant
        eprosima::amlip::dds::Participant participant("ManualTestParticipant");

        logUser(AMLIPCPP_MANUAL_TEST, "Created Participant: " << participant << ". Creating Reader for getting Id...");

        // Create Reader to get id of the target
        eprosima::fastdds::dds::DataReaderQos qos;
        qos = eprosima::amlip::dds::Reader<eprosima::amlip::types::AmlipIdDataType>::default_datareader_qos();
        qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
        qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
        qos.history().kind = eprosima::fastdds::dds::HistoryQosPolicyKind::KEEP_ALL_HISTORY_QOS;

        std::shared_ptr<eprosima::amlip::dds::Reader<eprosima::amlip::types::AmlipIdDataType>> reader =
                participant.create_reader<eprosima::amlip::types::AmlipIdDataType>("manual_test_topic", qos);

        logUser(AMLIPCPP_MANUAL_TEST, "Created Reader. Waiting for Id...");

        // Wait to receive target id
        reader->wait_data_available();

        logUser(AMLIPCPP_MANUAL_TEST, "Received Data with target Id. Reading Id...");

        // Read target id
        eprosima::amlip::types::AmlipIdDataType target_id = reader->read();

        logUser(AMLIPCPP_MANUAL_TEST, "Read Id: " << target_id << ". Creating Direct Writer...");

        // Create Writer
        std::shared_ptr<eprosima::amlip::dds::DirectWriter<eprosima::amlip::types::AmlipIdDataType>> writer =
                participant.create_direct_writer<eprosima::amlip::types::AmlipIdDataType>("manual_test_topic");

        logUser(AMLIPCPP_MANUAL_TEST, "Created Direct Writer. Waiting to match with TargetedReader...");

        // Wait for matching
        writer->wait_match(target_id);
        // Wait a bit to let the reader do the match
        std::this_thread::sleep_for(std::chrono::milliseconds(250));

        logUser(AMLIPCPP_MANUAL_TEST, "Matched with Reader. Seding data...");

        // Send data
        eprosima::amlip::types::AmlipIdDataType data("TESTDATA");
        writer->write(target_id, data);

        logUser(AMLIPCPP_MANUAL_TEST, "Direct Writer has sent message: " << data << ". Destroying entities...");
    }

    logUser(AMLIPCPP_MANUAL_TEST, "Finishing Manual Test DirectWriter execution.");

    return 0;
}
