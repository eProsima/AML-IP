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
"""Test AMLIP ID Data Type import."""

import amlip_swig


def test_id_creation():
    """Test creation of AmlipIdDataType."""
    # Create not defined Id
    new_id = amlip_swig.AmlipIdDataType()
    assert (not new_id.is_defined())

    # Create defined Id
    new_defined_id = amlip_swig.AmlipIdDataType('SomeTestName')
    assert (new_defined_id.is_defined())
