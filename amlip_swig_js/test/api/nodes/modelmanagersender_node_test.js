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

// Test that create an ModelManagerSenderNode does not give any error.
test('Create ModelManagerSenderNode', () => {
    console.log('TEST: Create ModelManagerSenderNode')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const id = new amlip_swig_js.AmlipIdDataType('modelmanagersender_node_test')
        const modelmanagersenderNode = new amlip_swig_js.ModelManagerSenderNodeJS(id);
        // If the ModelManagerSenderNode is created without throwing an error, the test passes
        assert(true, 'ModelManagerSenderNode is created without errors')
        console.log('ModelManagerSenderNode is created without errors')
        assert(modelmanagersenderNode instanceof amlip_swig_js.ModelManagerSenderNodeJS, 'modelmanagersenderNode is an instance of ModelManagerSenderNodeJS')
        console.log('modelmanagersenderNode is an instance of ModelManagerSenderNodeJS')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating ModelManagerSenderNode:', error)
        assert(false, 'Error creating ModelManagerSenderNode')
    }
});

test('Create ModelManagerSenderNode with domain', () => {
    console.log('TEST: Create ModelManagerSenderNode with domain')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const id = new amlip_swig_js.AmlipIdDataType('modelmanagersender_node_test')
        const modelmanagersenderNode = new amlip_swig_js.ModelManagerSenderNodeJS(id, 166);
        // If the ModelManagerSenderNode is created without throwing an error, the test passes
        assert(true, 'ModelManagerSenderNode is created with domain without errors')
        console.log('ModelManagerSenderNode is created with domain without errors')
        assert(modelmanagersenderNode instanceof amlip_swig_js.ModelManagerSenderNodeJS, 'modelmanagersenderNode is an instance of ModelManagerSenderNodeJS')
        console.log('modelmanagersenderNode is an instance of ModelManagerSenderNodeJS')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating ModelManagerSenderNode with domain:', error)
        assert(false, 'Error creating ModelManagerSenderNode with domain')
    }
});
