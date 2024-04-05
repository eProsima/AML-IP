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

// Test that create an MainNode does not give any error.
test('Create MainNode', () => {
    console.log('TEST: Create MainNode')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const mainNode = new amlip_swig_js.MainNode('main_node_test');
        // If the MainNode is created without throwing an error, the test passes
        assert(true, 'MainNode is created without errors')
        console.log('MainNode is created without errors')
        assert(mainNode instanceof amlip_swig_js.MainNode, 'mainNode is an instance of MainNode')
        console.log('mainNode is an instance of MainNode')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating MainNode:', error)
        assert(false, 'Error creating MainNode')
    }
});

test('Create MainNode with domain', () => {
    console.log('TEST: Create MainNode with domain')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const mainNode = new amlip_swig_js.MainNode('main_node_test', 166);
        // If the MainNode is created without throwing an error, the test passes
        assert(true, 'MainNode is created with domain without errors')
        console.log('MainNode is created with domain without errors')
        assert(mainNode instanceof amlip_swig_js.MainNode, 'mainNode is an instance of MainNode')
        console.log('mainNode is an instance of MainNode')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating MainNode with domain:', error)
        assert(false, 'Error creating MainNode with domain')
    }
});
