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

from amlip_py.node.AsyncEdgeNode import AsyncEdgeNode, InferenceListenerLambda
from amlip_py.types.InferenceDataType import InferenceDataType

# Variable to wait to the inference
waiter = BooleanWaitHandler(True, False)

# Domain ID
DOMAIN_ID = 10


def inference_received(
        inference,
        task_id,
        server_id):
    print(f'Data received from server: {server_id}\n'
          f' with id: {task_id}\n'
          f' inference: {inference.to_string()}')
    waiter.open()


def main():
    """Execute main routine."""
    # Initialize random number generator
    # TODO

    # Create node
    print('Starting Manual Test Async Edge Node Py execution. Creating Node...')
    edge_node = AsyncEdgeNode(
        'PyTestAsyncEdgeNode',
        listener=InferenceListenerLambda(inference_received),
        domain=DOMAIN_ID)

    # Create data
    print(f'Node created: {edge_node.get_id()}. Creating data...')
    data_str = '<Inference Data In Py String Async [LISTENER]>'
    inference_data = InferenceDataType(data_str)

    # Sending data
    print(f'Data created with string: {inference_data}. Sending request...')
    task_id = edge_node.request_inference(inference_data)

    print(f'Request sent with task id: {task_id}. Waiting inference...')

    # Wait to received solution
    waiter.wait()

    print('Finishing Manual Test Async Edge Node Py execution.')


# Call main in program execution
if __name__ == '__main__':
    main()
