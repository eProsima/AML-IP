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

console.log('Starting Manual Test Main Node JS execution. Creating Node...')
const main_node = new amlip_swig_js.MainNode('main_node_test');
console.log('Node created.')

class JobDataType extends amlip_swig_js.GenericDataType {}
job_data = new JobDataType('HELLO WORLD')
console.log('Job data is: ' + job_data.to_string())

console.log('Sending request...')
const solution = main_node.request_job_solution(job_data)

console.log('Solution receiced from server. Deserializing to string...')
console.log('Solution deserialized is: ' + solution.to_string())

console.log('Finishing Manual Test Main Node JS execution.')
