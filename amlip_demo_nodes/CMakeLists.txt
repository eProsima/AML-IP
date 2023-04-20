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
cmake_minimum_required(VERSION 3.15)

# Project name
project(amlip_demo_nodes)

# Find dependencies
find_package(cpp_utils)
find_package(amlip_cpp)
find_package(ddsrouter_core )

# C++ nodes
add_executable(computing_node computing_node.cpp)
target_link_libraries(computing_node amlip_cpp cpp_utils)
install(
    TARGETS
        computing_node
    DESTINATION
        bin/
)

# Python nodes
install(
    PROGRAMS
        main_node.py
    DESTINATION
        bin/
)
