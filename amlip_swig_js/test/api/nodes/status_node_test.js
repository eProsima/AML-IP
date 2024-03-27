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

// Test that create an StatusNode does not give any error.
test('Create StatusNode', () => {
    console.log('TEST: Create StatusNode')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const statusNode = new amlip_swig_js.StatusNode('status_node_test');
        // If the StatusNode is created without throwing an error, the test passes
        assert(true, 'StatusNode is created without errors')
        console.log('StatusNode is created without errors')
        assert(statusNode instanceof amlip_swig_js.StatusNode, 'statusNode is an instance of StatusNode')
        console.log('statusNode is an instance of StatusNode')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating StatusNode:', error)
        assert(false, 'Error creating StatusNode')
    }
});

test('Create StatusNode with domain', () => {
    console.log('TEST: Create StatusNode with domain')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const statusNode = new amlip_swig_js.StatusNode('status_node_test', 166);
        // If the StatusNode is created without throwing an error, the test passes
        assert(true, 'StatusNode is created with domain without errors')
        console.log('StatusNode is created with domain without errors')
        assert(statusNode instanceof amlip_swig_js.StatusNode, 'statusNode is an instance of StatusNode')
        console.log('statusNode is an instance of StatusNode')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating StatusNode with domain:', error)
        assert(false, 'Error creating StatusNode with domain')
    }
});
