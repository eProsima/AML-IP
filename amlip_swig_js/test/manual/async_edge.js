// Copyright 2024 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');

function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

async function main() {

    console.log('Starting Manual Test Async Edge Node JS execution. Creating Node...')
    const listener = new amlip_swig_js.InferenceSolutionListenerJS()

    const async_edge_node = new amlip_swig_js.AsyncEdgeNodeJS('async_edge_node_test', listener)
    console.log('Node created. Creating inference...')

    class InferenceDataType extends amlip_swig_js.GenericDataType {}
    data_str = '<INFERENCE DATA IN JS STRING [CUSTOM]>'
    const inference_data = new InferenceDataType(data_str)
    console.log('Inference data is: ' + inference_data.to_string())

    console.log('Sending request...')
    const task_id = async_edge_node.request_inference(inference_data)
    console.log('Request sent with task id: ' + task_id + ' . Waiting for inference...')

    while (amlip_swig_js.block_edge_) {
        console.log('Waiting for inference...')
        await sleep(1000)
    }

    console.log('Finishing Manual Test Async Edge Node JS execution.')
}

main();
