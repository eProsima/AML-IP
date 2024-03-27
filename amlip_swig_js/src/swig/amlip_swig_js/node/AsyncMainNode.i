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

////////////////////////////////////////////////////////
// Binding for class AsyncMainNode
////////////////////////////////////////////////////////

// Import parent class
%import(module="amlip_swig_js") "amlip_cpp/node/ParentNode.hpp";

%ignore eprosima::amlip::node::SolutionListener;

%unique_ptr(SolutionListenerJS)

%{
#include <amlip_cpp/node/workload_distribution/AsyncMainNode.hpp>
%}

// Include the class interfaces
%include <amlip_cpp/node/workload_distribution/AsyncMainNode.hpp>

%inline %{

bool block_main_ = true;

class SolutionListenerJS : public eprosima::amlip::node::SolutionListener
{
public:

    SolutionListenerJS()
    {
        // Do nothing
    }

    virtual ~SolutionListenerJS()
    {
        // Do nothing
    }

    void solution_received(
            const eprosima::amlip::types::JobSolutionDataType& solution,
            const eprosima::amlip::types::TaskId& task_id,
            const eprosima::amlip::types::AmlipIdDataType& server_id) override
    {
        std::cout << "Solution received for task : " << task_id
                                                     << " answered from server : " << server_id
                                                     << " . Solution : " << solution << " ." << std::endl;
        block_main_ = false;
    }

};

// Transfers a unique_ptr in JavaScript to a shared_ptr in C++
class AsyncMainNodeJS : public eprosima::amlip::node::AsyncMainNode
{
public:
    AsyncMainNodeJS(
            const char* name,
            std::unique_ptr<SolutionListenerJS>& listener,
            uint32_t domain_id)
        : eprosima::amlip::node::AsyncMainNode(name, std::move(listener), domain_id)
    {
        // Do nothing
    }

    AsyncMainNodeJS(
            const char* name,
            std::unique_ptr<SolutionListenerJS>& listener)
        : eprosima::amlip::node::AsyncMainNode(name, std::move(listener))
    {
        // Do nothing
    }
};
%}
