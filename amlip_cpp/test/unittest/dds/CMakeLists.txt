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
# PARTICIPANT TESTS
#####################################

set(TEST_NAME participantTest)

set(TEST_SOURCES
        participantTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/dds/DdsHandler.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/dds/network_utils/dds_qos.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/dds/Participant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/id/AmlipIdDataType.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/id/AmlipIdDataTypev1.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/InterfaceDataType.cpp
    )

set(TEST_LIST
    # Public API
        create_dummy_participant
        id
        get_name
        create_with_name
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
# DDS HANDLER TESTS
#####################################

set(TEST_NAME DdsHandlerTest)

set(TEST_SOURCES
        ddsHandlerTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/dds/DdsHandler.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/dds/Participant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/dds/network_utils/dds_qos.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/dds/network_utils/topic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/id/AmlipIdDataType.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/id/AmlipIdDataTypev1.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/InterfaceDataType.cpp
    )

set(TEST_LIST
    # Public API
        create_participant
        create_datawriter
        create_datareader
        fail_create_participant
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
# ENTITIES CREATION TESTS
#####################################

set(TEST_NAME EntitiesCreationTest)

set(TEST_SOURCES
        entitiesCreationTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/dds/DdsHandler.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/dds/network_utils/dds_qos.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/dds/Participant.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/dds/Writer.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/dds/network_utils/direct_write.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/dds/network_utils/multiservice.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/dds/network_utils/topic.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/id/AmlipIdDataType.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/id/AmlipIdDataTypev1.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/InterfaceDataType.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/multiservice/MsRequestDataType.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/multiservice/MsRequestDataTypev1.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/multiservice/MsReferenceDataType.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/multiservice/MsReferenceDataTypev1.cpp
    )

set(TEST_LIST
        create_participant
        create_writer
        create_reader
        create_directwriter
        create_targetedreader
        create_ms_client
        create_ms_server
        create_rpc_client
        create_rpc_server
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
