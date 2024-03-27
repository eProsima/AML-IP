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

// Test that create an ModelManagerReceiverNode does not give any error.
test('Create ModelManagerReceiverNode', () => {
    console.log('TEST: Create ModelManagerReceiverNode')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const id = new amlip_swig_js.AmlipIdDataType('modelmanagerreceiver_node_test')
        class ModelRequestDataType extends amlip_swig_js.GenericDataType {}
        const data = new ModelRequestDataType('modelRequestDataType_test');
        const modelmanagerreceiverNode = new amlip_swig_js.ModelManagerReceiverNodeJS(id, data);
        // If the ModelManagerReceiverNode is created without throwing an error, the test passes
        assert(true, 'ModelManagerReceiverNode is created without errors')
        console.log('ModelManagerReceiverNode is created without errors')
        assert(modelmanagerreceiverNode instanceof amlip_swig_js.ModelManagerReceiverNodeJS, 'modelmanagerreceiverNode is an instance of ModelManagerReceiverNodeJS')
        console.log('modelmanagerreceiverNode is an instance of ModelManagerReceiverNodeJS')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating ModelManagerReceiverNode:', error)
        assert(false, 'Error creating ModelManagerReceiverNode')
    }
});

test('Create ModelManagerReceiverNode with domain', () => {
    console.log('TEST: Create ModelManagerReceiverNode with domain')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const id = new amlip_swig_js.AmlipIdDataType('modelmanagerreceiver_node_test')
        class ModelRequestDataType extends amlip_swig_js.GenericDataType {}
        const data = new ModelRequestDataType('modelRequestDataType_test');
        const modelmanagerreceiverNode = new amlip_swig_js.ModelManagerReceiverNodeJS(id, data, 166);
        // If the ModelManagerReceiverNode is created without throwing an error, the test passes
        assert(true, 'ModelManagerReceiverNode is created with domain without errors')
        console.log('ModelManagerReceiverNode is created with domain without errors')
        assert(modelmanagerreceiverNode instanceof amlip_swig_js.ModelManagerReceiverNodeJS, 'modelmanagerreceiverNode is an instance of ModelManagerReceiverNodeJS')
        console.log('modelmanagerreceiverNode is an instance of ModelManagerReceiverNodeJS')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating ModelManagerReceiverNode with domain:', error)
        assert(false, 'Error creating ModelManagerReceiverNode with domain')
    }
});
