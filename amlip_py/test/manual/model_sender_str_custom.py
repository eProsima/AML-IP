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

from amlip_py.node.ModelManagerSenderNode import ModelManagerSenderNode, ModelReplier
from amlip_py.types.AmlipIdDataType import AmlipIdDataType
from amlip_py.types.ModelDataType import ModelDataType
from amlip_py.types.ModelSolutionDataType import ModelSolutionDataType
from amlip_py.types.ModelStatisticsDataType import ModelStatisticsDataType

# Domain ID
DOMAIN_ID = 166

# Variable to wait to the model request
waiter = BooleanWaitHandler(True, False)


class CustomModelReplier(ModelReplier):

    def fetch_model(
            self,
            model: ModelDataType) -> ModelSolutionDataType:
        solution = ModelSolutionDataType(model.to_string().upper())
        print(f'Model request received from client\n'
              f' model: {model.to_string()}\n'
              f' solution: {solution.to_string()}')

        waiter.open()

        return solution


def main():
    """Execute main routine."""

    id = AmlipIdDataType('ModelManagerSender')
    id.set_id([66, 66, 66, 66])

    # Create node
    print('Starting Manual Test Model Manager Sender Node Py execution. Creating Node...')
    model_sender_node = ModelManagerSenderNode(
        id=id,
        domain=DOMAIN_ID)

    print(f'Node created: {model_sender_node.get_id()}. '
          'Already processing models.')

    model_sender_node.update_statistics(
        'ModelManagerSenderStatistics',
        'hello world')

    model_sender_node.start(
        listener=CustomModelReplier())

    # Wait for the solution to be sent
    waiter.wait()

    model_sender_node.stop()

    print('Finishing Manual Test Model Manager Sender Node Py execution.')


# Call main in program execution
if __name__ == '__main__':
    main()
