# Copyright 2024 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
# Compile JavaScript wrapper with SWIG
###############################################################################

message(STATUS "Configuring AMLIP_SWIG wrapper for Javascript...")

set_source_files_properties(
        ${PROJECT_NAME}.i
        PROPERTIES CPLUSPLUS ON
        USE_TARGET_INCLUDE_DIRECTORIES TRUE
        SWIG_FLAGS -node
    )

swig_add_library(${PROJECT_NAME}
        TYPE SHARED
        LANGUAGE javascript
        SOURCES ${PROJECT_NAME}.i
    )

if(UNIX AND CMAKE_SIZEOF_VOID_P EQUAL 8)
    set_property(TARGET ${PROJECT_NAME} PROPERTY SWIG_COMPILE_DEFINITIONS SWIGWORDSIZE64)
endif()

target_link_libraries(${PROJECT_NAME}
        amlip_cpp
    )

SET (JAVASCRIPT_MODULE_PATH
    ${LIB_INSTALL_DIR}/node_modules/${PROJECT_NAME}
)

# Install
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}-targets
        DESTINATION ${JAVASCRIPT_MODULE_PATH}
    )
get_property(support_files TARGET ${PROJECT_NAME} PROPERTY SWIG_SUPPORT_FILES)
install(FILES ${support_files} DESTINATION ${JAVASCRIPT_MODULE_PATH})

export(TARGETS ${PROJECT_NAME} FILE ${PROJECT_BINARY_DIR}/cmake/config/${PROJECT_NAME}-targets.cmake)

install(EXPORT ${PROJECT_NAME}-targets
        DESTINATION ${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME}/
    )

# Create CMake config file.
include(CMakePackageConfigHelpers)

configure_package_config_file(
        ${cmake_utils_CONFIG_TEMPLATES_PATH}/library-Config.cmake.in
        ${PROJECT_BINARY_DIR}/cmake/config/${PROJECT_NAME}-config.cmake
        INSTALL_DESTINATION ${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME}
        PATH_VARS LIB_INSTALL_DIR
    )
write_basic_package_version_file(${PROJECT_BINARY_DIR}/cmake/config/${PROJECT_NAME}-config-version.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
    )
install(FILES ${PROJECT_BINARY_DIR}/cmake/config/${PROJECT_NAME}-config.cmake
        ${PROJECT_BINARY_DIR}/cmake/config/${PROJECT_NAME}-config-version.cmake
        DESTINATION ${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME}
    )
