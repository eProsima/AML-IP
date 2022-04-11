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

#include <dds/Reader.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

void ReaderListener::on_data_available(
        eprosima::fastdds::dds::DataReader* reader)
{
    if(callback_set_.load())
    {
        on_data_available_callback_();
    }
}

void ReaderListener::on_subscription_matched(
        eprosima::fastdds::dds::DataReader* reader,
        const eprosima::fastdds::dds::SubscriptionMatchedStatus& info)
{
    if (info.current_count_change > 0)
    {
        logDebug(AMLIP_WRITER, "Reader " << reader->guid() << " matched with Writer.");
    }
    else if (info.current_count_change < 0)
    {
        logDebug(AMLIP_WRITER, "Reader " << reader->guid() << " unmatched with Writer.");
    }
}

void ReaderListener::set_callback(std::function<void()> on_data_available_callback) noexcept
{
    on_data_available_callback_ = on_data_available_callback;
    callback_set_.store(true);
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */
