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

#include <gtest_aux.hpp>
#include <gtest/gtest.h>

#include <dds/DdsHandler.hpp>
#include <amlip_cpp/types/id/AmlipIdDataType.hpp>
#include <dds/Participant.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

namespace test {

class DdsHandlerMock : public DdsHandler
{
public:

    using DdsHandler::DdsHandler;

    using DdsHandler::participant_;
    using DdsHandler::datawriters_;
};

} /* namespace test */

/**
 * Create a DdsHandler with a Participant and change some input QoS to check
 * the ptr to the Participant has been created correctly
 */
TEST(DdsHandlerTest, create_participant)
{
    // Chech values
    DomainIdType domain_(42);
    std::string name_ = "__ParticipantTest";

    // Create domain
    DomainIdType domain(domain_);

    // Create QoS
    eprosima::fastdds::dds::DomainParticipantQos qos;

    qos.name(name_);

    // Create Handler with internal participant
    test::DdsHandlerMock handler(qos, domain);

    // Check the internal values
    ASSERT_EQ(handler.participant_->get_domain_id(), domain_);
    ASSERT_EQ(handler.participant_->get_qos().name().to_string(), name_);

    // Check that the Participant Properties are the modified qos
    const std::string* application_id =
            eprosima::fastrtps::rtps::PropertyPolicyHelper::find_property(handler.participant_->get_qos().properties(), "fastdds.application.id");
    ASSERT_NE(application_id, nullptr);
    ASSERT_EQ(application_id->compare("AML_IP"), 0);
}

/**
 * Create a DdsHandler and from it create a DataWriter.
 * Check it is created with correct QoS.
 */
TEST(DdsHandlerTest, create_datawriter)
{
    // Chech values
    std::string topic_name_ = "__TopicTest";
    // Set DataWriter QoS
    eprosima::fastdds::dds::DataWriterQos qos;

    qos = Writer<types::AmlipIdDataType>::default_datawriter_qos();

    qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    qos.ownership().kind = eprosima::fastdds::dds::OwnershipQosPolicyKind::EXCLUSIVE_OWNERSHIP_QOS;

    // Create in place listener
    eprosima::fastdds::dds::DataWriterListener listener;

    // Create Handler with internal participant
    test::DdsHandlerMock handler(eprosima::fastdds::dds::DomainParticipantQos(), DomainIdType(42));

    // Create DataWriter
    eprosima::utils::LesseePtr<eprosima::fastdds::dds::DataWriter> datawriter =
            handler.create_datawriter<types::AmlipIdDataType>(topic_name_, qos, &listener);

    // Check DataWriter internal values
    auto datawriter_locked = datawriter.lock();
    ASSERT_TRUE(datawriter_locked);
    ASSERT_EQ(datawriter_locked->get_topic()->get_name(), utils::topic_name_mangling(topic_name_));

    // Check that the Participant Properties are the modified qos
    const std::string* application_id =
            eprosima::fastrtps::rtps::PropertyPolicyHelper::find_property(datawriter_locked->get_qos().properties(), "fastdds.application.id");
    ASSERT_NE(application_id, nullptr);
    ASSERT_EQ(application_id->compare("AML_IP"), 0);

    qos.properties().properties().emplace_back("fastdds.application.id", "AML_IP", true);
    ASSERT_EQ(datawriter_locked->get_qos(), qos);

    ASSERT_EQ(datawriter_locked->get_listener(), &listener);
}

/**
 * Create a DdsHandler and from it create a DataReader.
 * Check it is created with correct QoS.
 */
TEST(DdsHandlerTest, create_datareader)
{
    // Chech values
    std::string topic_name_ = "__TopicTest";
    // Set DataReader QoS
    eprosima::fastdds::dds::DataReaderQos qos;

    qos = Reader<types::AmlipIdDataType>::default_datareader_qos();

    qos.durability().kind = eprosima::fastdds::dds::DurabilityQosPolicyKind::TRANSIENT_LOCAL_DURABILITY_QOS;
    qos.reliability().kind = eprosima::fastdds::dds::ReliabilityQosPolicyKind::RELIABLE_RELIABILITY_QOS;
    qos.ownership().kind = eprosima::fastdds::dds::OwnershipQosPolicyKind::EXCLUSIVE_OWNERSHIP_QOS;

    // Create in place listener
    eprosima::fastdds::dds::DataReaderListener listener;

    // Create Handler with internal participant
    test::DdsHandlerMock handler(eprosima::fastdds::dds::DomainParticipantQos(), DomainIdType(42));

    // Create DataReader
    eprosima::utils::LesseePtr<eprosima::fastdds::dds::DataReader> datareader =
            handler.create_datareader<types::AmlipIdDataType>(topic_name_, qos, &listener);

    // Check DataReader internal values
    auto datareader_locked = datareader.lock();
    ASSERT_TRUE(datareader_locked);
    ASSERT_EQ(datareader_locked->get_topicdescription()->get_name(), utils::topic_name_mangling(topic_name_));

    // Check that the Participant Properties are the modified qos
    const std::string* application_id =
            eprosima::fastrtps::rtps::PropertyPolicyHelper::find_property(datareader_locked->get_qos().properties(), "fastdds.application.id");
    ASSERT_NE(application_id, nullptr);
    ASSERT_EQ(application_id->compare("AML_IP"), 0);

    qos.properties().properties().emplace_back("fastdds.application.id", "AML_IP", true);
    ASSERT_EQ(datareader_locked->get_qos(), qos);


    ASSERT_EQ(datareader_locked->get_listener(), &listener);
}

/**
 * Create a DdsHandler with a Participant QoS that make the creation fail
 */
TEST(DdsHandlerTest, fail_create_participant)
{
    // Create QoS without transports
    eprosima::fastdds::dds::DomainParticipantQos qos;
    qos.transport().use_builtin_transports = false;

    // Create Handler with internal participant
    ASSERT_THROW(
        test::DdsHandlerMock(qos, DomainIdType(42)),
        eprosima::utils::InitializationException);
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

int main(
        int argc,
        char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
