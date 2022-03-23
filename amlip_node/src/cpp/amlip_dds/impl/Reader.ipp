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
 * @file Reader.ipp
 */

#ifndef AMLIP__SRC_CPP_AMLIPDDS_IMPL_READER_IPP
#define AMLIP__SRC_CPP_AMLIPDDS_IMPL_READER_IPP

namespace eprosima {
namespace amlip {
namespace dds {

template <typename T>
Reader<T>::Reader(
        const std::string& topic,
        std::shared_ptr<eprosima::fastdds::dds::DataReader> reader)
{

}

template <typename T>
Reader<T>::~Reader()
{

}

template <typename T>
void Reader<T>::wait_data_available(uint32_t timeout /* = 0 */)
{

}

template <typename T>
void Reader<T>::stop()
{

}

template <typename T>
T Reader<T>::read(void* data)
{

}

template <typename T>
bool Reader<T>::is_data_available()
{

}

template <typename T>
void Reader<T>::on_data_available(
        eprosima::fastdds::dds::DataReader* reader) override
{

}

template <typename T>
eprosima::fastdds::dds::DataWriterQos Reader<T>::default_datareader_qos_() const
{

}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP__SRC_CPP_AMLIPDDS_IMPL_READER_IPP */
