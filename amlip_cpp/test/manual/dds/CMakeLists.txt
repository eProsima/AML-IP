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

# NOTE: This test cannot be used linking the library cause it uses things that are not exported (it uses internal API)
set(TEST_SOURCES
    )
all_library_sources("${TEST_SOURCES}")

set(TEST_NAMES
        "reader"
        "writer"
        "targetedreader"
        "directwriter"
        "msclient"
        "msserver"
        "rpcclient"
        "rpcserver"
    )

foreach(TEST_NAME ${TEST_NAMES})

    set(EXECUTABLE_TEST_NAME
        "amlip_test_${TEST_NAME}")

    set(${TEST_NAME}_TEST_SOURCES
            "${TEST_NAME}.cpp"
            "${TEST_SOURCES}"
        )

    add_executable(
            ${EXECUTABLE_TEST_NAME}
            ${${TEST_NAME}_TEST_SOURCES}
        )

    target_include_directories(${EXECUTABLE_TEST_NAME} PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
        ${PROJECT_BINARY_DIR}/include
        ${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}
        ${PROJECT_SOURCE_DIR}/src/cpp
    )

    target_link_libraries(
            ${EXECUTABLE_TEST_NAME}
            ${MODULE_DEPENDENCIES}
        )

    if(LOG_INFO)
    target_compile_definitions(${EXECUTABLE_TEST_NAME}
        PRIVATE FASTDDS_ENFORCE_LOG_INFO
        PRIVATE HAVE_LOG_NO_INFO=0
        PRIVATE AMLIP_CPP_SOURCE
        )
    endif()

    unset(EXECUTABLE_TEST_NAME)

endforeach()
