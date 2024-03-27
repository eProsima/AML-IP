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

const assert = require('assert');

// Test that create an AsyncComputingNodeJS does not give any error.
test('Create AsyncComputingNodeJS', () => {
    console.log('TEST: Create AsyncComputingNodeJS')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const replier = new amlip_swig_js.JobReplierJS()
        const asynccomputingNode = new amlip_swig_js.AsyncComputingNodeJS('async_computing_node_test', replier);
        // If the AsyncComputingNodeJS is created without throwing an error, the test passes
        assert(true, 'AsyncComputingNodeJS is created without errors')
        console.log('AsyncComputingNodeJS is created without errors')
        assert(asynccomputingNode instanceof amlip_swig_js.AsyncComputingNodeJS, 'asynccomputingNode is an instance of AsyncComputingNodeJS')
        console.log('asynccomputingNode is an instance of AsyncComputingNodeJS')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating AsyncComputingNodeJS:', error)
        assert(false, 'Error creating AsyncComputingNodeJS')
    }
});

test('Create AsyncComputingNodeJS with domain', () => {
    console.log('TEST: Create AsyncComputingNodeJS with domain')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const replier = new amlip_swig_js.JobReplierJS()
        const asynccomputingNode = new amlip_swig_js.AsyncComputingNodeJS('async_computing_node_test', replier, 166);
        // If the AsyncComputingNodeJS is created without throwing an error, the test passes
        assert(true, 'AsyncComputingNodeJS is created with domain without errors')
        console.log('AsyncComputingNodeJS is created with domain without errors')
        assert(asynccomputingNode instanceof amlip_swig_js.AsyncComputingNodeJS, 'asynccomputingNode is an instance of AsyncComputingNodeJS')
        console.log('asynccomputingNode is an instance of AsyncComputingNodeJS')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating AsyncComputingNodeJS with domain:', error)
        assert(false, 'Error creating AsyncComputingNodeJS with domain')
    }
});
