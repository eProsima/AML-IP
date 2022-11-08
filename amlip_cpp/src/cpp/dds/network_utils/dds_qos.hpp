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
 * @file dds_qos.hpp
 */

#ifndef AMLIPCPP__SRC_CPP_DDS_NETWORKUTILS_DDSQOS_HPP
#define AMLIPCPP__SRC_CPP_DDS_NETWORKUTILS_DDSQOS_HPP

#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/subscriber/qos/SubscriberQos.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>

namespace eprosima {
namespace amlip {
namespace dds {
namespace utils {

fastdds::dds::DomainParticipantQos default_domain_participant_qos();

fastdds::dds::PublisherQos default_publisher_qos();

fastdds::dds::DataWriterQos default_datawriter_qos();

fastdds::dds::SubscriberQos default_subscriber_qos();

fastdds::dds::DataReaderQos default_datareader_qos();

} /* namespace utils */
} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIPCPP__SRC_CPP_DDS_NETWORKUTILS_DDSQOS_HPP */
