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

// Test that create an AsyncEdgeNode does not give any error.
test('Create AsyncEdgeNode', () => {
    console.log('TEST: Create AsyncEdgeNode')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const listener = new amlip_swig_js.InferenceSolutionListenerJS()
        const asyncedgeNode = new amlip_swig_js.AsyncEdgeNodeJS('async_edge_node_test', listener);
        // If the AsyncEdgeNode is created without throwing an error, the test passes
        assert(true, 'AsyncEdgeNode is created without errors')
        console.log('AsyncEdgeNode is created without errors')
        assert(asyncedgeNode instanceof amlip_swig_js.AsyncEdgeNodeJS, 'asyncedgeNode is an instance of AsyncEdgeNode')
        console.log('asyncedgeNode is an instance of AsyncEdgeNode')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating AsyncEdgeNode:', error)
        assert(false, 'Error creating AsyncEdgeNode')
    }
});

test('Create AsyncEdgeNode with domain', () => {
    console.log('TEST: Create AsyncEdgeNode with domain')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const listener = new amlip_swig_js.InferenceSolutionListenerJS()
        const asyncedgeNode = new amlip_swig_js.AsyncEdgeNodeJS('async_edge_node_test', listener, 166);
        // If the AsyncEdgeNode is created without throwing an error, the test passes
        assert(true, 'AsyncEdgeNode is created with domain without errors')
        console.log('AsyncEdgeNode is created with domain without errors')
        assert(asyncedgeNode instanceof amlip_swig_js.AsyncEdgeNodeJS, 'asyncedgeNode is an instance of AsyncEdgeNode')
        console.log('asyncedgeNode is an instance of AsyncEdgeNode')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating AsyncEdgeNode with domain:', error)
        assert(false, 'Error creating AsyncEdgeNode with domain')
    }
});
