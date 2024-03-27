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

console.log('Starting Manual Test Computing Node JS execution. Creating Node...')

const computing_node = new amlip_swig_js.ComputingNode('computing_node_test');
console.log('Node created.')

var job_listener = new amlip_swig_js.JobListenerJS()
console.log('JobListenerJS created.')

console.log('Processing job...')
computing_node.process_job(job_listener)

console.log('Solution sent to client.')

console.log('Finishing Manual Test Computing Node JS execution.')
