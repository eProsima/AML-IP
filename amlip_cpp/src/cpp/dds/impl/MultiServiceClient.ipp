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
 * @file MultiServiceClient.ipp
 */

#ifndef AMLIP__SRC_CPP_AMLIPCPP_DDS_IMPL_READER_IPP
#define AMLIP__SRC_CPP_AMLIPCPP_DDS_IMPL_READER_IPP

#include <ddsrouter_utils/exception/InconsistencyException.hpp>

namespace eprosima {
namespace amlip {
namespace dds {

template <typename T>
MultiServiceClient<T>::MultiServiceClient(
        const std::string& topic,
        ddsrouter::utils::LesseePtr<DdsHandler> dds_handler)
    : topic_(topic)
{
    auto dds_handler_locked = dds_handler.lock_with_exception();

    // Initialize entities
    // TODO
}

} /* namespace dds */
} /* namespace amlip */
} /* namespace eprosima */

#endif /* AMLIP__SRC_CPP_AMLIPCPP_DDS_IMPL_READER_IPP */
