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

# Required CMake minimum version
cmake_minimum_required(VERSION 3.5)

# Project name
project(amlip_workload_distribution_demo)

# Find dependencies
find_package(cpp_utils)
find_package(ddspipe_core)
find_package(ddspipe_participants)
find_package(ddsrouter_core)
find_package(amlip_cpp)

# C++ executables
add_executable(computing_node_sync computing_node_sync.cpp)
target_link_libraries(computing_node_sync amlip_cpp cpp_utils)
install(
    TARGETS
    computing_node_sync
    DESTINATION
        bin/
)

# Python executables
install(
    PROGRAMS
        main_node_sync.py
        main_node_async.py
        computing_node_async.py
        status_node.py
    DESTINATION
        bin/
)
