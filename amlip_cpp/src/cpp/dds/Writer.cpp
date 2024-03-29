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
 * @file Writer.cpp
 */

#include <cpp_utils/Log.hpp>

#include <dds/Writer.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

WriterListener::WriterListener()
    : matched_readers_(0)
{
}

WriterListener::~WriterListener()
{
    writer_match_waiter_.disable();
}

void WriterListener::on_publication_matched(
        eprosima::fastdds::dds::DataWriter* writer,
        const eprosima::fastdds::dds::PublicationMatchedStatus& info)
{
    if (info.current_count_change > 0)
    {
        logDebug(AMLIPCPP_WRITER, "Writer " << writer->guid() << " matched with Reader.");
        increase_match_();
    }
    else if (info.current_count_change < 0)
    {
        logDebug(AMLIPCPP_WRITER, "Writer " << writer->guid() << " unmatched with Reader.");
        decrease_match_();
    }
}

uint32_t WriterListener::readers_matched() const noexcept
{
    return matched_readers_.load();
}

eprosima::utils::event::AwakeReason WriterListener::wait_match(
        const eprosima::utils::Duration_ms& timeout /* = 0 */)
{
    return writer_match_waiter_.wait(timeout);
}

void WriterListener::increase_match_() noexcept
{
    matched_readers_++;

    if (matched_readers_ > 0)
    {
        writer_match_waiter_.open();
    }
}

void WriterListener::decrease_match_() noexcept
{
    matched_readers_--;

    if (matched_readers_ == 0)
    {
        writer_match_waiter_.close();
    }
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */
