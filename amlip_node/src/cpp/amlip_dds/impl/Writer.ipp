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
 * @file Writer.ipp
 */

#ifndef AMLIP__SRC_CPP_AMLIPDDS_IMPL_WRITER_IPP
#define AMLIP__SRC_CPP_AMLIPDDS_IMPL_WRITER_IPP

namespace eprosima {
namespace amlip {
namespace dds {

template <typename T>
Writer::<T>Writer(
        const std::string& topic,
        std::shared_ptr<eprosima::fastdds::dds::DataWriter> reader)
{
}

template <typename T>
Writer::<T>~Writer()
{
}

template <typename T>
void Writer::<T>write(T data)
{
}

template <typename T>
eprosima::fastdds::dds::DataWriterQos Writer::<T>default_datawriter_qos_() const
{
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP__SRC_CPP_AMLIPDDS_IMPL_WRITER_IPP */
