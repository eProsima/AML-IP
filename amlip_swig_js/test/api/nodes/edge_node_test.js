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

// Test that create an EdgeNode does not give any error.
test('Create EdgeNode', () => {
    console.log('TEST: Create EdgeNode')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const edgeNode = new amlip_swig_js.EdgeNode('edge_node_test');
        // If the EdgeNode is created without throwing an error, the test passes
        assert(true, 'EdgeNode is created without errors')
        console.log('EdgeNode is created without errors')
        assert(edgeNode instanceof amlip_swig_js.EdgeNode, 'edgeNode is an instance of EdgeNode')
        console.log('edgeNode is an instance of EdgeNode')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating EdgeNode:', error)
        assert(false, 'Error creating EdgeNode')
    }
});
