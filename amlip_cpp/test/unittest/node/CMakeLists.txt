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

#####################################
# NODE CREATION TESTS
#####################################

set(TEST_NAME NodeCreationTest)

set(TEST_SOURCES
        nodeCreationTest.cpp
    )
all_library_sources(${TEST_SOURCES})

set(TEST_LIST
        create_parent
        create_status
        create_main
        create_computational
    )

set(TEST_EXTRA_LIBRARIES
        "${MODULE_FIND_PACKAGES}"
    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
    )

#####################################
# STATUS NODE TESTS
#####################################

set(TEST_NAME StatusNodeTest)

set(TEST_SOURCES
        statusNodeTest.cpp
    )
all_library_sources(${TEST_SOURCES})

set(TEST_LIST
        run_and_stop
        process_status_parent
        process_status_state
    )

set(TEST_EXTRA_LIBRARIES
        "${MODULE_FIND_PACKAGES}"
    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
    )

