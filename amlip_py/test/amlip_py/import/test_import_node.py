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

# Types
import amlip_py.types.AmlipIdDataType  # noqa: F401
import amlip_py.types.JobDataType  # noqa: F401
import amlip_py.types.JobSolutionDataType  # noqa: F401
import amlip_py.types.StatusDataType  # noqa: F401

# Nodes
import amlip_py.node.ComputingNode  # noqa: F401,I100,I202
import amlip_py.node.MainNode  # noqa: F401
import amlip_py.node.StatusNode  # noqa: F401
import amlip_py.node.AsyncMainNode  # noqa: F401


def test_status_creation():
    """Test creation of StatusNode."""
    _ = amlip_py.node.StatusNode.StatusNode


def test_main_creation():
    """Test creation of MainNode."""
    _ = amlip_py.node.MainNode.MainNode


def test_computing_creation():
    """Test creation of ComputingNode."""
    _ = amlip_py.node.ComputingNode.ComputingNode


def test_asyncmain_creation():
    """Test creation of AsyncMainNode."""
    _ = amlip_py.node.AsyncMainNode.AsyncMainNode
