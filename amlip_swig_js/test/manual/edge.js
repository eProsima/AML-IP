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

console.log('Starting Manual Test Edge Node JS execution. Creating Node...')
const edge_node = new amlip_swig_js.EdgeNode('edge_node_test');
console.log('Node created. Creating inference...')

class InferenceDataType extends amlip_swig_js.GenericDataType {}
inference_data = new InferenceDataType('HELLO WORLD')
console.log('Inference data is: ' + inference_data.to_string())

console.log('Sending request...')
const solution = edge_node.request_inference(inference_data)

console.log('Solution receiced from server. Deserializing to string...')
console.log('Solution deserialized is: ' + solution.to_string())

console.log('Finishing Manual Test Edge Node JS execution.')
