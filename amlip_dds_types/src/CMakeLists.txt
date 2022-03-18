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
# Compile lbrary
###############################################################################
# Project sources
file(GLOB_RECURSE SOURCES_FILES
        "*.c"
        "*.cpp"
        "*.cxx"
    )

# Project headers
if(MSVC)
    file(GLOB_RECURSE HEADERS_FILES
            ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h
            ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/*.hpp
            ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/*.hxx
        )
endif()

set(${PROJECT_NAME}_SOURCES
        ${SOURCES_FILES}
        ${HEADERS_FILES}
    )

# Create library
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR})

# Define Public API
target_compile_definitions(${PROJECT_NAME}
    PRIVATE
        ${SUBMODULE_PROJECT_MACROS}_SOURCE
    )

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

target_link_libraries(${PROJECT_NAME}
        ${SUBMODULE_PROJECT_DEPENDENCIES}
    )

###############################################################################
# Packaging
###############################################################################
# Install headers
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
        DESTINATION ${INCLUDE_INSTALL_DIR}
        COMPONENT headers
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hpp"
        PATTERN "*.ipp"
    )

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}-targets
    RUNTIME DESTINATION ${BIN_INSTALL_DIR}
    LIBRARY DESTINATION ${LIB_INSTALL_DIR}
    ARCHIVE DESTINATION ${LIB_INSTALL_DIR}
    COMPONENT libraries
    )

install(EXPORT ${PROJECT_NAME}-targets
    DESTINATION ${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME}
    COMPONENT cmake
    )

###############################################################################
# Create CMake package config file
###############################################################################
include(CMakePackageConfigHelpers)

configure_package_config_file(${PROJECT_SOURCE_DIR}/cmake/packaging/Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    INSTALL_DESTINATION ${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME}
    PATH_VARS BIN_INSTALL_DIR INCLUDE_INSTALL_DIR LIB_INSTALL_DIR
    )
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
    )
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
    DESTINATION ${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME}
    COMPONENT cmake
    )

set(CPACK_COMPONENT_LIBRARIES_DISPLAY_NAME "Libraries" PARENT_SCOPE)
set(CPACK_COMPONENT_LIBRARIES_DESCRIPTION "eProsima ${PROJECT_NAME} libraries" PARENT_SCOPE)
