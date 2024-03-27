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

console.log('Starting Manual Test Inference Node JS execution. Creating Node...')

const inference_node = new amlip_swig_js.InferenceNode('inference_node_test');
console.log('Node created.')

var inference_listener = new amlip_swig_js.InferenceListenerJS()
console.log('InferenceListenerJS created.')

console.log('Processing inference...')
const client_id = inference_node.process_inference(inference_listener)

console.log('Solution sent to client ' + client_id + '.')

console.log('Finishing Manual Test Inference Node JS execution.')
