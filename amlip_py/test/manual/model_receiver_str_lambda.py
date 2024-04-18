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

import pickle as pkl

from py_utils.wait.BooleanWaitHandler import BooleanWaitHandler

from amlip_py.node.ModelManagerReceiverNode import ModelManagerReceiverNode
from amlip_py.types.AmlipIdDataType import AmlipIdDataType
from amlip_py.types.ModelReplyDataType import ModelReplyDataType
from amlip_py.types.ModelRequestDataType import ModelRequestDataType
from amlip_py.types.ModelStatisticsDataType import ModelStatisticsDataType

# Domain ID
DOMAIN_ID = 166

# Variable to wait to the model reply
waiter_statistics = BooleanWaitHandler(True, False)
waiter_model = BooleanWaitHandler(True, False)


def statistics_received(
        statistics: ModelStatisticsDataType) -> bool:

    data = pkl.loads(bytes(statistics.to_vector()))

    print('\n\nStatistics received: \n')
    print(data)
    print('\n')

    if (float(data['size']) < 100):
        print('Publish request.\n')
        waiter_statistics.open()
        return True

    return False


def model_received(
        model: ModelReplyDataType) -> bool:

    print('\nReply received:\n')
    print(model.to_string())
    print('\n')

    waiter_model.open()

    return True


def main():
    """Execute main routine."""

    # Create request
    data = ModelRequestDataType('MobileNet V1')

    id = AmlipIdDataType('ModelManagerReceiver')
    id.set_id([66, 66, 66, 66])

    # Create node
    print('Starting Manual Test Model Manager Receiver Node Py execution. Creating Node...')
    model_receiver_node = ModelManagerReceiverNode(
        id=id,
        data=data,
        domain=DOMAIN_ID)

    print(f'Node created: {model_receiver_node.get_id()}. '
          'Already processing models.')

    model_receiver_node.start(
        callback_statistics=statistics_received,
        callback_model=model_received)

    # Wait statistics
    waiter_statistics.wait()

    # do something...
    # decide to request the model
    model_receiver_node.request_model()

    # Wait model
    waiter_model.wait()

    model_receiver_node.stop()

    print('Finishing Manual Test Model Manager Sender Node Py execution.')


# Call main in program execution
if __name__ == '__main__':
    main()
