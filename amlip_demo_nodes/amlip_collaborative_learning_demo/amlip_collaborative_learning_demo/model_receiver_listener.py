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

from amlip_py.node.ModelManagerReceiverNode import ModelManagerReceiverNode, ModelListenerLambda
from amlip_py.types.AmlipIdDataType import AmlipIdDataType
from amlip_py.types.ModelReplyDataType import ModelReplyDataType
from amlip_py.types.ModelRequestDataType import ModelRequestDataType
from amlip_py.types.ModelStatisticsDataType import ModelStatisticsDataType

# Domain ID
DOMAIN_ID = 166

# Variable to wait for the statistics
waiter = BooleanWaitHandler(True, False)
# Variable to store the server id of the statistics
server_id = None


def statistics_received(
        statistics: ModelStatisticsDataType):

    print(f'Statistics received: {statistics.to_string()}')

    global server_id
    server_id = statistics.server_id()

    waiter.open()


def model_received(
        model: ModelReplyDataType) -> bool:

    print(f'Model reply received from server\n'
          f' solution: {model.to_string()}')

    return True


def main():
    """Execute main routine."""

    # Create request
    data = ModelRequestDataType('MobileNet V1')

    id = AmlipIdDataType('ModelManagerReceiver')
    id.set_id([15, 25, 35, 45])

    # Create node
    print('Starting Manual Test Model Manager Receiver Node Py execution. Creating Node...')
    model_receiver_node = ModelManagerReceiverNode(
        id=id,
        data=data,
        domain=DOMAIN_ID)

    print(f'Node created: {model_receiver_node.get_id()}. '
          'Already processing models.')

    model_receiver_node.start(
        listener=ModelListenerLambda(statistics_received, model_received))

    # Wait statistics
    waiter.wait()

    # Request model
    model_receiver_node.request_model(server_id)

    model_receiver_node.stop()

    print('Finishing Manual Test Model Manager Receiver Node Py execution.')


# Call main in program execution
if __name__ == '__main__':
    main()
