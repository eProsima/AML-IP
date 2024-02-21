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

// Test that importing amlip_swig_js does not give any error.

const assert = require('assert');

try {
    require('/home/irenebm/eprosima/annapurna/AML-IP-ws/install/amlip_swig_js/lib/node_modules/amlip_swig_js/amlip_swig_js.so');
} catch (error) {
    console.error('Error importing amlip_swig_js:', error);
    assert(false, 'Error importing amlip_swig_js')
}

// try {
//     require('/home/irenebm/eprosima/annapurna/AML-IP-ws/install/amlip_swig_js/lib/node_modules/amlip_swig_js/amlip_swig_js.so');
//     // If the module imports without throwing an error, the test passes
//     expect(true).toBe(true);
// } catch (error) {
//     // If an error occurs during import, fail the test and log the error
//     console.error('Error importing amlip_swig_js:', error);
//     // Fail the test explicitly
//     expect(true).toBe(false);
// }
