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

    console.log('Starting Manual Test ModelManagerReceiver Node JS execution. Creating Node...')
    const id = new amlip_swig_js.AmlipIdDataType('modelmanagerreceiver_node_test')
    class ModelRequestDataType extends amlip_swig_js.GenericDataType {}
    const data = new ModelRequestDataType('modelRequestDataType_test');
    const model_manager_receiver = new amlip_swig_js.ModelManagerReceiverNodeJS(id, data, 166);
    console.log('Node created. Already processing models...')

    const listener = new amlip_swig_js.ModelListenerJS()

    model_manager_receiver.start(listener)

    while (amlip_swig_js.block_modelmanagerreceiver_) {
        console.log('Waiting...')
        await sleep(1000)
    }

    model_manager_receiver.stop()

    console.log('Finishing Manual Test ModelManagerReceiver Node JS execution.')

}

main();
