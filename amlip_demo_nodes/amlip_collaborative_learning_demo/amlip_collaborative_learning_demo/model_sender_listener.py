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

from amlip_py.node.ModelManagerSenderNode import ModelManagerSenderNode, ModelReplierLambda
from amlip_py.types.AmlipIdDataType import AmlipIdDataType
from amlip_py.types.ModelReplyDataType import ModelReplyDataType
from amlip_py.types.ModelRequestDataType import ModelRequestDataType

# Domain ID
DOMAIN_ID = 166

# Variable to wait to the model request
waiter = BooleanWaitHandler(True, False)


def fetch_model(
        request: ModelRequestDataType) -> ModelReplyDataType:

    reply = ModelReplyDataType(request.to_string().upper())

    print(f'Model request received from client\n'
          f' request: {request.to_string()}\n'
          f' reply: {reply.to_string()}')

    waiter.open()

    return reply


def main():
    """Execute main routine."""

    id = AmlipIdDataType('ModelManagerSender')
    id.set_id([10, 20, 30, 40])

    # Create node
    print('Starting Manual Test Model Manager Sender Node Py execution. Creating Node...')
    model_sender_node = ModelManagerSenderNode(
        id=id,
        domain=DOMAIN_ID)

    print(f'Node created: {model_sender_node.get_id()}. '
          'Already processing models.')

    model_sender_node.publish_statistics(
        'ModelManagerSenderStatistics',
        'hello world')

    model_sender_node.start(
        listener=ModelReplierLambda(fetch_model))

    # Wait for the solution to be sent
    waiter.wait()

    model_sender_node.stop()

    print('Finishing Manual Test Model Manager Sender Node Py execution.')


# Call main in program execution
if __name__ == '__main__':
    main()
