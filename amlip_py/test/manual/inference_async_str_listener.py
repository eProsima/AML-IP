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

import signal

from amlip_py.node.AsyncInferenceNode import AsyncInferenceNode, InferenceReplierLambda
from amlip_py.types.InferenceSolutionDataType import InferenceSolutionDataType

# Domain ID
DOMAIN_ID = 166


def process_inference(
        inference,
        task_id,
        client_id):
    inference_solution = InferenceSolutionDataType(inference.to_string().lower())
    print(f'Data received from client: {client_id}\n'
          f' with id: {task_id}\n'
          f' job: {inference.to_string()}\n'
          f' inference: {inference_solution.to_string()}')
    return inference_solution


def main():
    """Execute main routine."""
    # Initialize random number generator
    # TODO

    # Create node
    print('Starting Manual Test Async Inference Node Py execution. Creating Node...')
    inference_node = AsyncInferenceNode(
        'PyTestAsyncInferenceNode',
        listener=InferenceReplierLambda(process_inference),
        domain=DOMAIN_ID)

    # Create job data
    print(f'Node created: {inference_node.get_id()}. '
          'Already processing inferences. Waiting SIGINT (C^)...')

    inference_node.run()

    def handler(signum, frame):
        pass
    signal.signal(signal.SIGINT, handler)
    signal.pause()

    inference_node.stop()

    print('Finishing Manual Test Async Inference Node Py execution.')


# Call main in program execution
if __name__ == '__main__':
    main()
