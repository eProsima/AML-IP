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

    console.log('Starting Manual Test Async Main Node JS execution. Creating Node...')
    const listener = new amlip_swig_js.SolutionListenerJS()

    const async_main_node = new amlip_swig_js.AsyncMainNodeJS('async_main_node_test', listener)
    console.log('Node created. Creating job...')

    class JobDataType extends amlip_swig_js.GenericDataType {}
    data_str = '<JOB DATA IN JS STRING [CUSTOM]>'
    const job_data = new JobDataType(data_str)
    console.log('Job data is: ' + job_data.to_string())

    console.log('Sending request...')
    const task_id = async_main_node.request_job_solution(job_data)
    console.log('Request sent with task id: ' + task_id + ' . Waiting for solution...')

    while (amlip_swig_js.block_main_) {
        console.log('Waiting for solution...')
        await sleep(1000)
    }

    console.log('Finishing Manual Test Async Main Node JS execution.')
}

main();
