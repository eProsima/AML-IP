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

# Add subdirectory with labels for tests
add_subdirectory(labels)

# Add subdirectory with tests
add_subdirectory(blackbox)
add_subdirectory(integration)
add_subdirectory(manual)
add_subdirectory(unittest)

# Add subdirectory with resources for tests
add_subdirectory(resources)
