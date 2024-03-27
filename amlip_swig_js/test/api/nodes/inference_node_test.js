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

// Test that create an InferenceNode does not give any error.
test('Create InferenceNode', () => {
    console.log('TEST: Create InferenceNode')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const inferenceNode = new amlip_swig_js.InferenceNode('inference_node_test');
        // If the InferenceNode is created without throwing an error, the test passes
        assert(true, 'InferenceNode is created without errors')
        console.log('InferenceNode is created without errors')
        assert(inferenceNode instanceof amlip_swig_js.InferenceNode, 'inferenceNode is an instance of InferenceNode')
        console.log('inferenceNode is an instance of InferenceNode')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating InferenceNode:', error)
        assert(false, 'Error creating InferenceNode')
    }
});
