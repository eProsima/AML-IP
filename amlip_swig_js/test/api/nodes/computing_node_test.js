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

// Test that create an ComputingNode does not give any error.
test('Create ComputingNode', () => {
    console.log('TEST: Create ComputingNode')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const computingNode = new amlip_swig_js.ComputingNode('computing_node_test');
        // If the ComputingNode is created without throwing an error, the test passes
        assert(true, 'ComputingNode is created without errors')
        console.log('ComputingNode is created without errors')
        assert(computingNode instanceof amlip_swig_js.ComputingNode, 'computingNode is an instance of ComputingNode')
        console.log('computingNode is an instance of ComputingNode')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating ComputingNode:', error)
        assert(false, 'Error creating ComputingNode')
    }
});

test('Create ComputingNode with domain', () => {
    console.log('TEST: Create ComputingNode with domain')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const computingNode = new amlip_swig_js.ComputingNode('computing_node_test', 166);
        // If the ComputingNode is created without throwing an error, the test passes
        assert(true, 'ComputingNode is created with domain without errors')
        console.log('ComputingNode is created with domain without errors')
        assert(computingNode instanceof amlip_swig_js.ComputingNode, 'computingNode is an instance of ComputingNode')
        console.log('computingNode is an instance of ComputingNode')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating ComputingNode with domain:', error)
        assert(false, 'Error creating ComputingNode with domain')
    }
});
