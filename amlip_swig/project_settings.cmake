# Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

###############################################################################
# Set settings for project amlip_swig for SWIG code
###############################################################################

set(MODULE_NAME
    amlip_swig)

set(MODULE_SUMMARY
    "SWIG library to create python bindings from amlip_cpp.")

set(MODULE_FIND_PACKAGES
        amlip_cpp
        cpp_utils
        ddspipe_core
        ddspipe_participants
        ddsrouter_core
    )

set(MODULE_DEPENDENCIES
        ${MODULE_FIND_PACKAGES}
    )
