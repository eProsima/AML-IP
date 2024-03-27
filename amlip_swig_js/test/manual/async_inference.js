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

const express = require('express');
const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');

const app = express();

const server = app.listen(() => {
        // Do nothing
});

console.log('Starting Manual Test Async Inference Node JS execution. Creating Node...')
const replier = new amlip_swig_js.InferenceReplierJS()
const async_inference_node = new amlip_swig_js.AsyncInferenceNodeJS('async_inference_node_test', replier);
console.log('Node created.')

console.log('Already processing inferences. Waiting SIGINT (C^)...')
async_inference_node.run()

process.once('SIGINT', () => {
    console.log('Received SIGINT. ');
    async_inference_node.stop()
    server.close(() => {
        console.log('Finishing Manual Test Async Inference Node JS execution.');
        process.exit(0);
    });
});
