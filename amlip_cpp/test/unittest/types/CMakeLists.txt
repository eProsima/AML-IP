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

#############
## AmlipId ##
#############

set(TEST_NAME amlipIdTest)

set(TEST_SOURCES
        amlipIdTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/InterfaceDataType.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/id/AmlipIdDataType.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/id/AmlipIdDataTypev1.cpp
    )

set(TEST_LIST
        create_id
        ids_not_equal
        serialization_deserialization
    )

set(TEST_EXTRA_LIBRARIES
        fastcdr
        fastrtps
        cpp_utils
    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
    )

#####################
## GenericDataType ##
#####################

set(TEST_NAME genericDataTypeTest)

set(TEST_SOURCES
        genericDataTypeTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/InterfaceDataType.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/id/AmlipIdDataType.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/id/AmlipIdDataTypev1.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/GenericDataType.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/GenericDataTypev1.cpp
    )

set(TEST_LIST
        generic_serialization_deserialization_int
        generic_serialization_deserialization_str
        generic_serialization_deserialization_amlipIdDataType
    )

set(TEST_EXTRA_LIBRARIES
        fastcdr
        fastrtps
        cpp_utils
    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
    )

#####################
## ModelStatistics ##
#####################

set(TEST_NAME modelStatisticsTest)

set(TEST_SOURCES
        modelStatisticsTest.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/InterfaceDataType.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/id/AmlipIdDataType.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/id/AmlipIdDataTypev1.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/model/ModelStatisticsDataType.cpp
        ${PROJECT_SOURCE_DIR}/src/cpp/types/model/ModelStatisticsDataTypev1.cpp
    )

set(TEST_LIST
        create_statistics
        statistics_with_long_data
    )

set(TEST_EXTRA_LIBRARIES
        fastcdr
        fastrtps
        cpp_utils
    )

set(TEST_NEEDED_SOURCES
    )

add_unittest_executable(
        "${TEST_NAME}"
        "${TEST_SOURCES}"
        "${TEST_LIST}"
        "${TEST_EXTRA_LIBRARIES}"
        "${TEST_NEEDED_SOURCES}"
    )
