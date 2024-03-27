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

    console.log('Starting Manual Test ModelManagerSender Node JS execution. Creating Node...')
    const id = new amlip_swig_js.AmlipIdDataType('modelmanagersender_node_test')
    const model_sender_node = new amlip_swig_js.ModelManagerSenderNodeJS(id, 166);    console.log('Node created. Already processing models...')

    model_sender_node.publish_statistics(
        'ModelManagerSenderStatistics',
        'hello world!'
    )

    console.log('Statistics published.')

    const listener = new amlip_swig_js.ModelReplierJS()

    model_sender_node.start(listener)

    while (amlip_swig_js.block_modelmanagersender_) {
        console.log('Waiting for request...')
        await sleep(1000)
    }

    model_sender_node.stop()

    console.log('Finishing Manual Test ModelManagerSender Node JS execution.')

}

main();
