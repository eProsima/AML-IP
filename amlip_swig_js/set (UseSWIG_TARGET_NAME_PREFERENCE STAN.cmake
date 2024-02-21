set (UseSWIG_TARGET_NAME_PREFERENCE STANDARD)
set(SWIG_JAVASCRIPT_EXTRA_FILE_EXTENSIONS ".js")

cmake_policy(GET CMP0078 target_name_policy)
if (target_name_policy STREQUAL "NEW")
    set (UseSWIG_TARGET_NAME_PREFERENCE STANDARD)
else()
    if (NOT target_name_policy)
        cmake_policy(GET_WARNING CMP0078 _cmp0078_warning)
        message(AUTHOR_WARNING "${_cmp0078_warning}\n")
    endif()
    if (NOT DEFINED UseSWIG_TARGET_NAME_PREFERENCE)
        set (UseSWIG_TARGET_NAME_PREFERENCE LEGACY)
    elseif (NOT UseSWIG_TARGET_NAME_PREFERENCE MATCHES "^(LEGACY|STANDARD)$")
        message (FATAL_ERROR "UseSWIG_TARGET_NAME_PREFERENCE: ${UseSWIG_TARGET_NAME_PREFERENCE}: invalid value. 'LEGACY' or 'STANDARD' is expected.")
    endif()
endif()

if (NOT DEFINED UseSWIG_MODULE_VERSION)
    set (UseSWIG_MODULE_VERSION 1)
elseif (NOT UseSWIG_MODULE_VERSION MATCHES "^(1|2)$")
    message (FATAL_ERROR "UseSWIG_MODULE_VERSION: ${UseSWIG_MODULE_VERSION}: invalid value. 1 or 2 is expected.")
endif()

# set (SWIG_MODULE_${PROJECT_NAME}_NOPROXY FALSE)
swig_module_initialize(${PROJECT_NAME} javascript)
string(TOUPPER "javascript" SWIG_MODULE_${PROJECT_NAME}_LANGUAGE)
string(TOLOWER "javascript" SWIG_MODULE_${PROJECT_NAME}_SWIG_LANGUAGE_FLAG)

# if (NOT DEFINED SWIG_MODULE_${PROJECT_NAME}_NOPROXY)
#     set (SWIG_MODULE_${PROJECT_NAME}_NOPROXY FALSE)
# endif()
# if ("-noproxy" IN_LIST CMAKE_SWIG_FLAGS)
#     set (SWIG_MODULE_${PROJECT_NAME}_NOPROXY TRUE)
# endif ()
# set (SWIG_MODULE_${PROJECT_NAME}_NOPROXY FALSE)
# if (SWIG_MODULE_${PROJECT_NAME}_NOPROXY AND
#     NOT ("-noproxy" IN_LIST CMAKE_SWIG_FLAGS OR "-noproxy" IN_LIST SWIG_MODULE_${PROJECT_NAME}_EXTRA_FLAGS))
#     list (APPEND SWIG_MODULE_${PROJECT_NAME}_EXTRA_FLAGS "-noproxy")
# endif()
if(SWIG_MODULE_${PROJECT_NAME}_LANGUAGE STREQUAL "UNKNOWN")
    message(FATAL_ERROR "SWIG Error: Language javascript not found")
elseif(SWIG_MODULE_${PROJECT_NAME}_LANGUAGE STREQUAL "PERL" AND
       NOT "-shadow" IN_LIST SWIG_MODULE_${PROJECT_NAME}_EXTRA_FLAGS)
    list(APPEND SWIG_MODULE_${PROJECT_NAME}_EXTRA_FLAGS "-shadow")
endif()

# if (TARGET ${PROJECT_NAME})
#     # a target with same name is already defined.
#     # call NOW add_library command to raise the most useful error message
#     add_library(${PROJECT_NAME})
#     return()
# endif()

# compute real target name.
set(target_name "${PROJECT_NAME}")

set (workingdir "${CMAKE_CURRENT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${target_name}.dir")
# set special variable to pass extra information to command SWIG_ADD_SOURCE_TO_MODULE
# which cannot be changed due to legacy compatibility
set (SWIG_WORKING_DIR "${workingdir}")
set (SWIG_TARGET_NAME "${target_name}")

if (CMAKE_SWIG_OUTDIR)
    set (outputdir "${CMAKE_SWIG_OUTDIR}")
else()
    if (UseSWIG_MODULE_VERSION VERSION_GREATER 1)
        set (outputdir "${workingdir}/javascript.files")
    else()
        set (outputdir "${CMAKE_CURRENT_BINARY_DIR}")
    endif()
endif()

if (SWIG_OUTFILE_DIR)
    set (outfiledir "${SWIG_OUTFILE_DIR}")
else()
    if (CMAKE_SWIG_OUTDIR)
        set (outfiledir "${outputdir}")
    else()
        set (outfiledir "${workingdir}")
    endif()
endif()

# set again, locally, predefined variables to ensure compatibility
# with command SWIG_ADD_SOURCE_TO_MODULE
set(CMAKE_SWIG_OUTDIR "${outputdir}")
set(SWIG_OUTFILE_DIR "${outfiledir}")

# See if the user has specified source extensions for swig files?
if (NOT DEFINED SWIG_SOURCE_FILE_EXTENSIONS)
    # Assume the default (*.i) file extension for Swig source files
    set(SWIG_SOURCE_FILE_EXTENSIONS ".i")
endif()

if (CMAKE_GENERATOR MATCHES "Make|Ninja|Xcode|Visual Studio (1[1-9]|[2-9][0-9])")
    # For Makefiles, Ninja, Xcode and Visual Studio generators,
    # use SWIG generated dependencies if requested
    if (NOT DEFINED SWIG_USE_SWIG_DEPENDENCIES)
        set (SWIG_USE_SWIG_DEPENDENCIES OFF)
    endif()
else()
    set (SWIG_USE_SWIG_DEPENDENCIES OFF)
endif()

# Generate a regex out of file extensions.
string(REGEX REPLACE "([$^.*+?|()-])" "\\\\\\1" swig_source_ext_regex "${SWIG_SOURCE_FILE_EXTENSIONS}")
list (JOIN swig_source_ext_regex "|" swig_source_ext_regex)
string (PREPEND swig_source_ext_regex "(")
string (APPEND swig_source_ext_regex ")$")

set(swig_dot_i_sources ${PROJECT_NAME}.i)
list(FILTER swig_dot_i_sources INCLUDE REGEX ${swig_source_ext_regex})
if (NOT swig_dot_i_sources)
    message(FATAL_ERROR "SWIG_ADD_LIBRARY: no SWIG interface files specified")
endif()
set(swig_other_sources ${PROJECT_NAME}.i)
list(REMOVE_ITEM swig_other_sources ${swig_dot_i_sources})

set(swig_generated_sources)
set(swig_generated_timestamps)
set(swig_generated_outdirs "${outputdir}")
list(LENGTH swig_dot_i_sources swig_sources_count)
if (swig_sources_count GREATER "1")
    # option -interface cannot be used
    set(SWIG_USE_INTERFACE FALSE)
else()
    set(SWIG_USE_INTERFACE TRUE)
endif()
foreach(swig_it IN LISTS swig_dot_i_sources)
    SWIG_ADD_SOURCE_TO_MODULE(${PROJECT_NAME} swig_generated_source "${swig_it}")
    list (APPEND swig_generated_sources "${swig_generated_source}")
    if(swig_timestamp)
        list (APPEND swig_generated_timestamps "${swig_timestamp}")
    endif()
    get_source_file_property(swig_source_file_outdir "${swig_it}" OUTPUT_DIR)
    if (swig_source_file_outdir)
        list (APPEND swig_generated_outdirs "${swig_source_file_outdir}")
    endif()
endforeach()
list(REMOVE_DUPLICATES swig_generated_outdirs)
set_property (DIRECTORY APPEND PROPERTY
    ADDITIONAL_CLEAN_FILES ${swig_generated_sources} ${swig_generated_timestamps})
if (UseSWIG_MODULE_VERSION VERSION_GREATER 1)
    set_property (DIRECTORY APPEND PROPERTY ADDITIONAL_CLEAN_FILES ${swig_generated_outdirs})
endif()

add_library(${target_name}
    SHARED
    ${swig_generated_sources}
    ${swig_other_sources})
if(swig_generated_timestamps)
    # see IMPLICIT_DEPENDS above
    add_custom_target(${PROJECT_NAME}_swig_compilation DEPENDS ${swig_generated_timestamps})
    add_dependencies(${target_name} ${PROJECT_NAME}_swig_compilation)
endif()

# assume empty prefix because we expect the module to be dynamically loaded
set_target_properties (${target_name} PROPERTIES PREFIX "")
set_target_properties (${target_name} PROPERTIES SUFFIX ".js")

# target property SWIG_SUPPORT_FILES_DIRECTORY specify output directories of support files
set_property (TARGET ${target_name} PROPERTY SWIG_SUPPORT_FILES_DIRECTORY ${swig_generated_outdirs})
# target property SWIG_SUPPORT_FILES lists principal proxy support files
if (NOT SWIG_MODULE_${PROJECT_NAME}_NOPROXY)
    string(TOUPPER "javascript" swig_uppercase_language)
    set(swig_all_support_files)
    foreach (swig_it IN LISTS SWIG_${swig_uppercase_language}_EXTRA_FILE_EXTENSIONS)
        set (swig_support_files ${swig_generated_sources})
        list (FILTER swig_support_files INCLUDE REGEX ".*${swig_it}$")
        list(APPEND swig_all_support_files ${swig_support_files})
    endforeach()
    if (swig_all_support_files)
        list(REMOVE_DUPLICATES swig_all_support_files)
    endif()
    set_property (TARGET ${target_name} PROPERTY SWIG_SUPPORT_FILES ${swig_all_support_files})
endif()

# to ensure legacy behavior, export some variables
set (SWIG_MODULE_${PROJECT_NAME}_LANGUAGE "${SWIG_MODULE_${PROJECT_NAME}_LANGUAGE}" PARENT_SCOPE)
set (SWIG_MODULE_${PROJECT_NAME}_SWIG_LANGUAGE_FLAG "${SWIG_MODULE_${PROJECT_NAME}_SWIG_LANGUAGE_FLAG}" PARENT_SCOPE)
set (SWIG_MODULE_${PROJECT_NAME}_REAL_NAME "${target_name}" PARENT_SCOPE)
set (SWIG_MODULE_${PROJECT_NAME}_NOPROXY "${SWIG_MODULE_${PROJECT_NAME}_NOPROXY}" PARENT_SCOPE)
set (SWIG_MODULE_${PROJECT_NAME}_EXTRA_FLAGS "${SWIG_MODULE_${PROJECT_NAME}_EXTRA_FLAGS}" PARENT_SCOPE)
# the last one is a bit crazy but it is documented, so...
# NOTA: works as expected if only ONE input file is specified
set (swig_generated_file_fullname "${swig_generated_file_fullname}" PARENT_SCOPE)

SET (JAVASCRIPT_MODULE_PATH
        ${LIB_INSTALL_DIR}/node_modules/${PROJECT_NAME}
    )

# target_include_directories(${PROJECT_NAME} PRIVATE)
target_link_libraries(${PROJECT_NAME} amlip_cpp)

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
