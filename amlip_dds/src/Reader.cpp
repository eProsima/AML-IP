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
 * @file Reader.cpp
 */

#include <iostream>

#include <amlip_dds_types/HelloWorld.h>
#include <amlip_dds_types/HelloWorldPubSubTypes.h>

#include <amlip_dds/Reader.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

Reader::Reader(std::string topic_name)
    : participant_(nullptr)
    , topic_(nullptr)
    , subscriber_(nullptr)
    , reader_(nullptr)
    , running_(false)
    , topic_name_(topic_name)
{
    std::cout << "Creating Reader in topic " << topic_name_ << "." << std::endl;
}

Reader::~Reader()
{
    std::cout << "Destroying ReaderNode in topic." << std::endl;
}

void Reader::start()
{
    std::cout << "Starting Reader in topic " << topic_name_ << "." << std::endl;
    running_.store(true);
}

void Reader::stop()
{
    std::cout << "Stopping Reader in topic " << topic_name_ << "." << std::endl;
    running_.store(false);
}

void Reader::on_data_available(
        fastdds::dds::DataReader*)
{
    // TODO
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */
