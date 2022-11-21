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
"""Test Status Data Type import."""

import amlip_swig
from amlip_swig import NodeKind_discovery, StateKind_running


def test_status_creation():
    """Test creation of StatusDataType."""
    new_status = amlip_swig.StatusDataType()
    assert (not new_status.id().is_defined())


def test_status_internal_info():
    """Test NodeKind object inside of StatusDataType."""
    status = amlip_swig.StatusDataType(
        amlip_swig.AmlipIdDataType('TestName'),
        NodeKind_discovery,
        StateKind_running
    )

    assert (status.id().name() == 'TestName')
    assert (status.node_kind() == NodeKind_discovery)
    assert (status.state() == StateKind_running)
