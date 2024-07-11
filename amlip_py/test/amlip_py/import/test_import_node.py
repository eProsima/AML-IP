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

# Nodes
import amlip_py.node.ComputingNode  # noqa: F401,I100,I202
import amlip_py.node.MainNode  # noqa: F401
import amlip_py.node.StatusNode  # noqa: F401
import amlip_py.node.AsyncMainNode  # noqa: F401
import amlip_py.node.AsyncComputingNode  # noqa: F401
import amlip_py.node.ModelManagerReceiverNode  # noqa: F401
import amlip_py.node.ModelManagerSenderNode  # noqa: F401
import amlip_py.node.ClientNode  # noqa: F401
import amlip_py.node.ServerNode  # noqa: F401
import amlip_py.node.RepeaterNode  # noqa: F401
import amlip_py.node.FiwareNode  # noqa: F401

def test_computing_node():
    """Test creation of ComputingNode."""
    _ = amlip_py.node.ComputingNode.ComputingNode('test_node')


def test_main_node():
    """Test creation of MainNode."""
    _ = amlip_py.node.MainNode.MainNode('test_node')


def test_status_node():
    """Test creation of StatusNode."""
    _ = amlip_py.node.StatusNode.StatusNode('test_node')


def test_model_manager_receiver_node():
    """Test creation of ModelManagerReceiverNode."""
    id = amlip_py.types.AmlipIdDataType.AmlipIdDataType()
    data = amlip_py.types.ModelRequestDataType.ModelRequestDataType()
    _ = amlip_py.node.ModelManagerReceiverNode.ModelManagerReceiverNode(id, data)


def test_model_manager_sender_node():
    """Test creation of ModelManagerSenderNode."""
    id = amlip_py.types.AmlipIdDataType.AmlipIdDataType()
    _ = amlip_py.node.ModelManagerSenderNode.ModelManagerSenderNode(id)
