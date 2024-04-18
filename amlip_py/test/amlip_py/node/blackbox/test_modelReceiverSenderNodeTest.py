# Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

from py_utils.wait.BooleanWaitHandler import BooleanWaitHandler

from amlip_py.node.ModelManagerReceiverNode import ModelManagerReceiverNode, ModelListener
from amlip_py.node.ModelManagerSenderNode import ModelManagerSenderNode, ModelReplier
from amlip_py.types.AmlipIdDataType import AmlipIdDataType
from amlip_py.types.ModelReplyDataType import ModelReplyDataType
from amlip_py.types.ModelRequestDataType import ModelRequestDataType
from amlip_py.types.ModelStatisticsDataType import ModelStatisticsDataType

# Domain ID
DOMAIN_ID = 166

# Variable to wait to the model request
waiter_statistics = BooleanWaitHandler(True, False)
waiter_model = BooleanWaitHandler(True, False)


class CustomModelReplier(ModelReplier):

    def fetch_model(
            self,
            model: ModelRequestDataType) -> ModelReplyDataType:

        solution = ModelReplyDataType(model.to_string().upper())

        print(f'Model request received from client\n'
              f' model: {model.to_string()}\n'
              f' solution: {solution.to_string()}')

        return solution


class CustomModelListener(ModelListener):

    def statistics_received(
            self,
            statistics: ModelStatisticsDataType) -> bool:

        waiter_statistics.open()

        return True

    def model_received(
            self,
            model: ModelReplyDataType) -> bool:

        print(f'Model reply received from server\n'
              f' solution: {model.to_string()}')

        waiter_model.open()

        return True


def test_one_model_receiver_one_model_sender():
    """
    Create 1 Model Manager Sender Node and 1 Model Manager Receiver Node.

    The
    """
    # Create request
    data = ModelRequestDataType('MobileNet V1')

    id_receiver = AmlipIdDataType('ModelManagerReceiver')
    id_receiver.set_id([66, 66, 66, 66])

    id_sender = AmlipIdDataType('ModelManagerSender')
    id_sender.set_id([66, 66, 66, 66])

    # Create nodes
    print('Starting Manual Test Model Manager Receiver Node Py execution. Creating Node...')
    model_receiver_node = ModelManagerReceiverNode(
        id=id_receiver,
        data=data,
        domain=DOMAIN_ID)

    print(f'Node created: {model_receiver_node.get_id()}. '
          'Already processing models.')

    print('Starting Manual Test Model Manager Sender Node Py execution. Creating Node...')
    model_sender_node = ModelManagerSenderNode(
        id=id_sender,
        domain=DOMAIN_ID)

    print(f'Node created: {model_sender_node.get_id()}. '
          'Already processing models.')

    model_sender_node.publish_statistics(
        'ModelManagerSenderStatistics',
        'hello world')

    model_receiver_node.start(
        listener=CustomModelListener())

    model_sender_node.start(
        listener=CustomModelReplier())

    # Wait statistics
    waiter_statistics.wait()

    # do something...
    # decide to request the model
    model_receiver_node.request_model()

    # Wait model
    waiter_model.wait()

    model_receiver_node.stop()
    model_sender_node.stop()

    print('Finishing Manual Test Model Manager Sender Node Py execution.')
