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

// Test that create an AsyncMainNode does not give any error.
test('Create AsyncMainNode', () => {
    console.log('TEST: Create AsyncMainNode')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const listener = new amlip_swig_js.SolutionListenerJS()
        const asyncmainNode = new amlip_swig_js.AsyncMainNodeJS('async_main_node_test', listener);
        // If the AsyncMainNode is created without throwing an error, the test passes
        assert(true, 'AsyncMainNode is created without errors')
        console.log('AsyncMainNode is created without errors')
        assert(asyncmainNode instanceof amlip_swig_js.AsyncMainNodeJS, 'asyncmainNode is an instance of AsyncMainNode')
        console.log('asyncmainNode is an instance of MainNode')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating AsyncMainNode:', error)
        assert(false, 'Error creating AsyncMainNode')
    }
});

test('Create AsyncMainNode with domain', () => {
    console.log('TEST: Create AsyncMainNode with domain')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const listener = new amlip_swig_js.SolutionListenerJS()
        const asyncmainNode = new amlip_swig_js.AsyncMainNodeJS('async_main_node_test', listener, 166);
        // If the AsyncMainNode is created without throwing an error, the test passes
        assert(true, 'AsyncMainNode is created with domain without errors')
        console.log('AsyncMainNode is created with domain without errors')
        assert(asyncmainNode instanceof amlip_swig_js.AsyncMainNodeJS, 'asyncmainNode is an instance of AsyncMainNode')
        console.log('asyncmainNode is an instance of AsyncMainNode')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating AsyncMainNode with domain:', error)
        assert(false, 'Error creating AsyncMainNode with domain')
    }
});
