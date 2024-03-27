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

// Test that create an AsyncInferenceNodeJS does not give any error.
test('Create AsyncInferenceNodeJS', () => {
    console.log('TEST: Create AsyncInferenceNodeJS')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const replier = new amlip_swig_js.InferenceReplierJS()
        const asyncinferenceNode = new amlip_swig_js.AsyncInferenceNodeJS('async_inference_node_test', replier);
        // If the AsyncInferenceNodeJS is created without throwing an error, the test passes
        assert(true, 'AsyncInferenceNodeJS is created without errors')
        console.log('AsyncInferenceNodeJS is created without errors')
        assert(asyncinferenceNode instanceof amlip_swig_js.AsyncInferenceNodeJS, 'asyncinferenceNode is an instance of AsyncInferenceNodeJS')
        console.log('asyncinferenceNode is an instance of AsyncInferenceNodeJS')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating AsyncInferenceNodeJS:', error)
        assert(false, 'Error creating AsyncInferenceNodeJS')
    }
});

test('Create AsyncInferenceNodeJS with domain', () => {
    console.log('TEST: Create AsyncInferenceNodeJS with domain')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const replier = new amlip_swig_js.InferenceReplierJS()
        const asyncinferenceNode = new amlip_swig_js.AsyncInferenceNodeJS('async_inference_node_test', replier, 166);
        // If the AsyncInferenceNodeJS is created without throwing an error, the test passes
        assert(true, 'AsyncInferenceNodeJS is created with domain without errors')
        console.log('AsyncInferenceNodeJS is created with domain without errors')
        assert(asyncinferenceNode instanceof amlip_swig_js.AsyncInferenceNodeJS, 'asyncinferenceNode is an instance of AsyncInferenceNodeJS')
        console.log('asyncinferenceNode is an instance of AsyncInferenceNodeJS')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating AsyncInferenceNodeJS with domain:', error)
        assert(false, 'Error creating AsyncInferenceNodeJS with domain')
    }
});
