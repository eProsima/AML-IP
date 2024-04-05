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

// Test that create an AmlipIdDataType does not give any error.
test('Create AmlipIdDataType', () => {
    console.log('TEST: Create AmlipIdDataType')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const amlipIdDataType = new amlip_swig_js.AmlipIdDataType();
        // If the AmlipIdDataType is created without throwing an error, the test passes
        assert(true, 'AmlipIdDataType is created without errors')
        console.log('AmlipIdDataType is created without errors')
        assert(amlipIdDataType instanceof amlip_swig_js.AmlipIdDataType, 'amlipIdDataType is an instance of AmlipIdDataType')
        console.log('amlipIdDataType is an instance of AmlipIdDataType')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating AmlipIdDataType:', error);
        assert(false, 'Error creating AmlipIdDataType')
    }
});

test('Create AmlipIdDataType with name', () => {
    console.log('TEST: Create AmlipIdDataType with name')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const amlipIdDataType = new amlip_swig_js.AmlipIdDataType('amlipIdDataType_test');
        // If the AmlipIdDataType is created without throwing an error, the test passes
        assert(true, 'AmlipIdDataType is created with name without errors')
        console.log('AmlipIdDataType is created with name without errors')
        assert(amlipIdDataType instanceof amlip_swig_js.AmlipIdDataType, 'amlipIdDataType is an instance of AmlipIdDataType')
        console.log('amlipIdDataType is an instance of AmlipIdDataType')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating AmlipIdDataType with name:', error);
        assert(false, 'Error creating AmlipIdDataType with name')
    }
});

// Test that create an GenericDataType does not give any error.
test('Create GenericDataType', () => {
    console.log('TEST: Create GenericDataType')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const genericDataType = new amlip_swig_js.GenericDataType();
        // If the GenericDataType is created without throwing an error, the test passes
        assert(true, 'GenericDataType is created without errors')
        console.log('GenericDataType is created without errors')
        assert(genericDataType instanceof amlip_swig_js.GenericDataType, 'genericDataType is an instance of GenericDataType')
        console.log('genericDataType is an instance of GenericDataType')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating GenericDataType:', error);
        assert(false, 'Error creating GenericDataType')
    }
});

test('Create GenericDataType with name', () => {
    console.log('TEST: Create GenericDataType with name')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const genericDataType = new amlip_swig_js.GenericDataType('genericDataType_test');
        // If the GenericDataType is created without throwing an error, the test passes
        assert(true, 'GenericDataType is created with name without errors')
        console.log('GenericDataType is created with name without errors')
        assert(genericDataType instanceof amlip_swig_js.GenericDataType, 'genericDataType is an instance of GenericDataType')
        console.log('genericDataType is an instance of GenericDataType')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating GenericDataType with name:', error);
        assert(false, 'Error creating GenericDataType with name')
    }
});

// Test that create an StatusDataType does not give any error.
test('Create StatusDataType', () => {
    console.log('TEST: Create StatusDataType')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const statusDataType = new amlip_swig_js.StatusDataType();
        // If the statusDataType is created without throwing an error, the test passes
        assert(true, 'StatusDataType is created without errors')
        console.log('StatusDataType is created without errors')
        assert(statusDataType instanceof amlip_swig_js.StatusDataType, 'statusDataType is an instance of StatusDataType')
        console.log('statusDataType is an instance of StatusDataType')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating StatusDataType:', error);
        assert(false, 'Error creating StatusDataType')
    }
});

// Test that create an GenericDataType does not give any error.
test('Create StatusDataType with id, node_kind and state', () => {
    console.log('TEST: Create StatusDataType with id, node_kind and state')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const id = new amlip_swig_js.AmlipIdDataType();
        const node_kind = amlip_swig_js.NodeKind_undetermined;
        const state = amlip_swig_js.StateKind_unknown;
        const statusDataType = new amlip_swig_js.StatusDataType(id, node_kind, state);
        // If the statusDataType is created without throwing an error, the test passes
        assert(true, 'StatusDataType is created with id, node_kind and state without errors')
        console.log('StatusDataType is created with id, node_kind and state without errors')
        assert(statusDataType instanceof amlip_swig_js.StatusDataType, 'statusDataType is an instance of StatusDataType')
        console.log('statusDataType is an instance of StatusDataType')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating StatusDataType with id, node_kind and state:', error);
        assert(false, 'Error creating StatusDataType with id, node_kind and state')
    }
});

// Test that create a JobDataType does not give any error.
test('Create JobDataType', () => {
    console.log('TEST: Create JobDataType')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        class JobDataType extends amlip_swig_js.GenericDataType {}
        const jobDataType = new JobDataType();
        // If the jobDataType is created without throwing an error, the test passes
        assert(true, 'JobDataType is created without errors')
        console.log('JobDataType is created without errors')
        assert(jobDataType instanceof amlip_swig_js.GenericDataType, 'jobDataType is an instance of GenericDataType')
        console.log('jobDataType is an instance of GenericDataType')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating JobDataType:', error);
        assert(false, 'Error creating JobDataType')
    }
});

test('Create JobDataType with name', () => {
    console.log('TEST: Create JobDataType with name')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        class JobDataType extends amlip_swig_js.GenericDataType {}
        const jobDataType = new JobDataType('jobDataType_test');
        // If the jobDataType is created without throwing an error, the test passes
        assert(true, 'JobDataType is created with name without errors')
        console.log('JobDataType is created with name without errors')
        assert(jobDataType instanceof amlip_swig_js.GenericDataType, 'jobDataType is an instance of GenericDataType')
        console.log('jobDataType is an instance of GenericDataType')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating JobDataType with name:', error);
        assert(false, 'Error creating JobDataType with name')
    }
});

// Test that create a JobSolutionDataType does not give any error.
test('Create JobSolutionDataType', () => {
    console.log('TEST: Create JobSolutionDataType')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        class JobSolutionDataType extends amlip_swig_js.GenericDataType {}
        const jobSolutionDataType = new JobSolutionDataType();
        // If the jobSolutionDataType is created without throwing an error, the test passes
        assert(true, 'JobSolutionDataType is created without errors')
        console.log('JobSolutionDataType is created without errors')
        assert(jobSolutionDataType instanceof amlip_swig_js.GenericDataType, 'jobSolutionDataType is an instance of GenericDataType')
        console.log('jobSolutionDataType is an instance of GenericDataType')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating JobSolutionDataType:', error);
        assert(false, 'Error creating JobSolutionDataType')
    }
});

test('Create JobSolutionDataType', () => {
    console.log('TEST: Create JobSolutionDataType with name')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        class JobSolutionDataType extends amlip_swig_js.GenericDataType {}
        const jobSolutionDataType = new JobSolutionDataType('jobSolutionDataType_test');
        // If the jobSolutionDataType is created without throwing an error, the test passes
        assert(true, 'JobSolutionDataType is created with name without errors')
        console.log('JobSolutionDataType is created with name without errors')
        assert(jobSolutionDataType instanceof amlip_swig_js.GenericDataType, 'jobSolutionDataType is an instance of GenericDataType')
        console.log('jobSolutionDataType is an instance of GenericDataType')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating JobSolutionDataType with name:', error);
        assert(false, 'Error creating JobSolutionDataType with name')
    }
});

// Test that create an InferenceDataType does not give any error.
test('Create InferenceDataType', () => {
    console.log('TEST: Create InferenceDataType')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        class InferenceDataType extends amlip_swig_js.GenericDataType {}
        const inferenceDataType = new InferenceDataType();
        // If the inferenceDataType is created without throwing an error, the test passes
        assert(true, 'InferenceDataType is created without errors')
        console.log('InferenceDataType is created without errors')
        assert(inferenceDataType instanceof amlip_swig_js.GenericDataType, 'inferenceDataType is an instance of GenericDataType')
        console.log('inferenceDataType is an instance of GenericDataType')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating InferenceDataType:', error);
        assert(false, 'Error creating InferenceDataType')
    }
});

test('Create InferenceDataType with name', () => {
    console.log('TEST: Create InferenceDataType with name')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        class InferenceDataType extends amlip_swig_js.GenericDataType {}
        const inferenceDataType = new InferenceDataType('inferenceDataType_test');
        // If the inferenceDataType is created without throwing an error, the test passes
        assert(true, 'InferenceDataType is created with name without errors')
        console.log('InferenceDataType is created with name without errors')
        assert(inferenceDataType instanceof amlip_swig_js.GenericDataType, 'inferenceDataType is an instance of GenericDataType')
        console.log('inferenceDataType is an instance of GenericDataType')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating InferenceDataType with name:', error);
        assert(false, 'Error creating InferenceDataType with name')
    }
});

// Test that create an InferenceSolutionDataType does not give any error.
test('Create InferenceSolutionDataType', () => {
    console.log('TEST: Create InferenceSolutionDataType')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        class InferenceSolutionDataType extends amlip_swig_js.GenericDataType {}
        const inferenceSolutionDataType = new InferenceSolutionDataType();
        // If the inferenceSolutionDataType is created without throwing an error, the test passes
        assert(true, 'InferenceSolutionDataType is created without errors')
        console.log('InferenceSolutionDataType is created without errors')
        assert(inferenceSolutionDataType instanceof amlip_swig_js.GenericDataType, 'inferenceSolutionDataType is an instance of GenericDataType')
        console.log('inferenceSolutionDataType is an instance of GenericDataType')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating InferenceSolutionDataType:', error);
        assert(false, 'Error creating InferenceSolutionDataType')
    }
});

test('Create InferenceSolutionDataType with name', () => {
    console.log('TEST: Create InferenceSolutionDataType with name')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        class InferenceSolutionDataType extends amlip_swig_js.GenericDataType {}
        const inferenceSolutionDataType = new InferenceSolutionDataType('inferenceSolutionDataType_test');
        // If the inferenceSolutionDataType is created without throwing an error, the test passes
        assert(true, 'InferenceSolutionDataType is created with name without errors')
        console.log('InferenceSolutionDataType is created with name without errors')
        assert(inferenceSolutionDataType instanceof amlip_swig_js.GenericDataType, 'inferenceSolutionDataType is an instance of GenericDataType')
        console.log('inferenceSolutionDataType is an instance of GenericDataType')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating InferenceSolutionDataType with name:', error);
        assert(false, 'Error creating InferenceSolutionDataType with name')
    }
});

// Test that create a ModelRequestDataType does not give any error.
test('Create ModelRequestDataType', () => {
    console.log('TEST: Create ModelRequestDataType')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        class ModelRequestDataType extends amlip_swig_js.GenericDataType {}
        const modelRequestDataType = new ModelRequestDataType();
        // If the modelRequestDataType is created without throwing an error, the test passes
        assert(true, 'ModelRequestDataType is created without errors')
        console.log('ModelRequestDataType is created without errors')
        assert(modelRequestDataType instanceof amlip_swig_js.GenericDataType, 'modelRequestDataType is an instance of GenericDataType')
        console.log('modelRequestDataType is an instance of GenericDataType')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating ModelRequestDataType:', error);
        assert(false, 'Error creating ModelRequestDataType')
    }
});

test('Create ModelRequestDataType with name', () => {
    console.log('TEST: Create ModelRequestDataType with name')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        class ModelRequestDataType extends amlip_swig_js.GenericDataType {}
        const modelRequestDataType = new ModelRequestDataType('modelRequestDataType_test');
        // If the modelRequestDataType is created without throwing an error, the test passes
        assert(true, 'ModelRequestDataType is created with name without errors')
        console.log('ModelRequestDataType is created with name without errors')
        assert(modelRequestDataType instanceof amlip_swig_js.GenericDataType, 'modelRequestDataType is an instance of GenericDataType')
        console.log('modelRequestDataType is an instance of GenericDataType')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating ModelRequestDataType with name:', error);
        assert(false, 'Error creating ModelRequestDataType with name')
    }
});

// Test that create a ModelReplyDataType does not give any error.
test('Create ModelReplyDataType', () => {
    console.log('TEST: Create ModelReplyDataType')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        class ModelReplyDataType extends amlip_swig_js.GenericDataType {}
        const modelReplyDataType = new ModelReplyDataType();
        // If the modelReplyDataType is created without throwing an error, the test passes
        assert(true, 'ModelReplyDataType is created without errors')
        console.log('ModelReplyDataType is created without errors')
        assert(modelReplyDataType instanceof amlip_swig_js.GenericDataType, 'modelReplyDataType is an instance of GenericDataType')
        console.log('modelReplyDataType is an instance of GenericDataType')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating ModelReplyDataType:', error);
        assert(false, 'Error creating ModelReplyDataType')
    }
});

test('Create ModelReplyDataType with name', () => {
    console.log('TEST: Create ModelReplyDataType with name')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        class ModelReplyDataType extends amlip_swig_js.GenericDataType {}
        const modelReplyDataType = new ModelReplyDataType('modelReplyDataType_test');
        // If the modelReplyDataType is created without throwing an error, the test passes
        assert(true, 'ModelReplyDataType is created with name without errors')
        console.log('ModelReplyDataType is created with name without errors')
        assert(modelReplyDataType instanceof amlip_swig_js.GenericDataType, 'modelReplyDataType is an instance of GenericDataType')
        console.log('modelReplyDataType is an instance of GenericDataType')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating ModelReplyDataType with name:', error);
        assert(false, 'Error creating ModelReplyDataType with name')
    }
});

// Test that create a ModelStatisticsDataType does not give any error.
test('Create ModelStatisticsDataType', () => {
    console.log('TEST: Create ModelStatisticsDataType')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const modelStatisticsDataType = new amlip_swig_js.ModelStatisticsDataType();
        // If the modelStatisticsDataType is created without throwing an error, the test passes
        assert(true, 'ModelStatisticsDataType is created without errors')
        console.log('ModelStatisticsDataType is created without errors')
        assert(modelStatisticsDataType instanceof amlip_swig_js.ModelStatisticsDataType, 'modelStatisticsDataType is an instance of ModelStatisticsDataType')
        console.log('modelStatisticsDataType is an instance of ModelStatisticsDataType')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating ModelStatisticsDataType:', error);
        assert(false, 'Error creating ModelStatisticsDataType')
    }
});

test('Create ModelStatisticsDataType  with name', () => {
    console.log('TEST: Create ModelStatisticsDataType with name')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const modelStatisticsDataType = new amlip_swig_js.ModelStatisticsDataType('modelStatisticsDataType_test');
        // If the modelStatisticsDataType is created without throwing an error, the test passes
        assert(true, 'ModelStatisticsDataType is created with name without errors')
        console.log('ModelStatisticsDataType is created with name without errors')
        assert(modelStatisticsDataType instanceof amlip_swig_js.ModelStatisticsDataType, 'modelStatisticsDataType is an instance of ModelStatisticsDataType')
        console.log('modelStatisticsDataType is an instance of ModelStatisticsDataType')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating ModelStatisticsDataType with name:', error);
        assert(false, 'Error creating ModelStatisticsDataType with name')
    }
});

test('Create ModelStatisticsDataType  with name and bytes', () => {
    console.log('TEST: Create ModelStatisticsDataType with name and bytes')
    try {
        const amlip_swig_js = require('/home/irenebm/eprosima/annapurna/AML-IP-ws/src/amlip/amlip_swig_js/build/Release/amlip_swig_js.node');
        const modelStatisticsDataType = new amlip_swig_js.ModelStatisticsDataType('modelStatisticsDataType_test', 'hello world');
        // If the modelStatisticsDataType is created without throwing an error, the test passes
        assert(true, 'ModelStatisticsDataType is created with name and bytes without errors')
        console.log('ModelStatisticsDataType is created with name and bytes without errors')
        assert(modelStatisticsDataType instanceof amlip_swig_js.ModelStatisticsDataType, 'modelStatisticsDataType is an instance of ModelStatisticsDataType')
        console.log('modelStatisticsDataType is an instance of ModelStatisticsDataType')
    } catch (error) {
        // If an error occurs during creation, fail the test and log the error
        console.error('Error creating ModelStatisticsDataType with name and bytes:', error);
        assert(false, 'Error creating ModelStatisticsDataType with name and bytes')
    }
});
