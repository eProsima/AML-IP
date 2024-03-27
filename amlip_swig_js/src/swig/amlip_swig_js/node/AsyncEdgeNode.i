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
// Binding for class AsyncEdgeNode
////////////////////////////////////////////////////////

// Import parent class
%import(module="amlip_swig_js") "amlip_cpp/node/ParentNode.hpp";

%ignore eprosima::amlip::node::InferenceSolutionListener;

%unique_ptr(InferenceSolutionListenerJS)

%{
#include <amlip_cpp/node/AsyncEdgeNode.hpp>
%}

// Include the class interfaces
%include <amlip_cpp/node/AsyncEdgeNode.hpp>

%inline %{

bool block_edge_ = true;

class InferenceSolutionListenerJS : public eprosima::amlip::node::InferenceSolutionListener
{
public:

    InferenceSolutionListenerJS()
    {
        // Do nothing
    }

    virtual ~InferenceSolutionListenerJS()
    {
        // Do nothing
    }

    void inference_received(
            const eprosima::amlip::types::InferenceSolutionDataType& solution,
            const eprosima::amlip::types::TaskId& task_id,
            const eprosima::amlip::types::AmlipIdDataType& server_id) override
    {
        std::cout << "Inference solution received for task : " << task_id
                                                     << " answered from server : " << server_id
                                                     << " . Solution : " << solution << " ." << std::endl;
        block_edge_ = false;
    }
};

// Transfers a unique_ptr in JavaScript to a shared_ptr in C++
class AsyncEdgeNodeJS : public eprosima::amlip::node::AsyncEdgeNode
{
public:

    AsyncEdgeNodeJS(
            const char* name,
            std::unique_ptr<InferenceSolutionListenerJS>& listener,
            uint32_t domain_id)
        : eprosima::amlip::node::AsyncEdgeNode(name, std::move(listener), domain_id)
    {
        // Do nothing
    }

    AsyncEdgeNodeJS(
            const char* name,
            std::unique_ptr<InferenceSolutionListenerJS>& listener)
        : eprosima::amlip::node::AsyncEdgeNode(name, std::move(listener))
    {
        // Do nothing
    }
};
%}
