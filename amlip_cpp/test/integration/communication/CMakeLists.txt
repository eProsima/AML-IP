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
# WRITER & READER COMMUNICATION TESTS
#####################################

set(TEST_NAME PublicationTest)

set(TEST_SOURCES
        publicationTest.cpp
    )

set(TEST_LIST
        communicate_reader_writer
        communicate_reader_writer_multiple_messages
        communicate_reader_multiple_writers
        communicate_multiple_readers_writer
        communicate_multiple_readers_multiple_writers
    )

set(TEST_NEEDED_SOURCES
    )

add_blackbox_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_NEEDED_SOURCES}"
    )

##################################################
# WRITER & READER DIRECT COMMUNICATION TESTS
##################################################

set(TEST_NAME DirectMessageTest)

set(TEST_SOURCES
        directMessageTest.cpp
    )

set(TEST_LIST
        communicate_writer_reader
        communicate_writer_reader_multiple_messages
        communicate_multiple_writers_reader
        communicate_writer_multiple_readers
    )

set(TEST_NEEDED_SOURCES
    )

add_blackbox_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_NEEDED_SOURCES}"
    )

##################################################
# MULTISERVICE COMMUNICATION TESTS
##################################################

set(TEST_NAME MultiServiceTest)

set(TEST_SOURCES
        multiserviceTest.cpp
    )

set(TEST_LIST
        communicate_service_one_on_one
        communicate_service_n_to_n
    )

set(TEST_NEEDED_SOURCES
    )

add_blackbox_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_NEEDED_SOURCES}"
    )
