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

console.log('Starting Manual Test Status Node JS execution. Creating Node...')
const status_node = new amlip_swig_js.StatusNode('status_node_test');
console.log('Node created.')

const listener = new amlip_swig_js.StatusListenerJS()

console.log('Listener created. Processing data asynchronously...')
status_node.process_status_async(listener)

console.log('Already processing status data. Waiting SIGINT (C^)...')

process.once('SIGINT', () => {
    console.log('Received SIGINT. ');
    server.close(() => {
        console.log('Finishing Manual Test Status Node JS execution.');
        process.exit(0);
    });
});
