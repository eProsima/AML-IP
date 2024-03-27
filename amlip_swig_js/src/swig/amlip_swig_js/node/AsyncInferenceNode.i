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
// Binding for class AsyncInferenceNode
////////////////////////////////////////////////////////

// Import parent class
%import(module="amlip_swig_js") "amlip_cpp/node/ParentNode.hpp";

%ignore eprosima::amlip::node::InferenceReplier;

%unique_ptr(InferenceReplierJS);

%{
#include <amlip_cpp/node/AsyncInferenceNode.hpp>
%}

// Include the class interfaces
%include <amlip_cpp/node/AsyncInferenceNode.hpp>

%inline %{

class InferenceReplierJS : public eprosima::amlip::node::InferenceReplier
{
public:

    InferenceReplierJS()
    {
        // Do nothing
    }

    virtual ~InferenceReplierJS()
    {
        // Do nothing
    }

    eprosima::amlip::types::InferenceSolutionDataType process_inference(
            const eprosima::amlip::types::InferenceDataType& inference,
            const eprosima::amlip::types::TaskId& task_id,
            const eprosima::amlip::types::AmlipIdDataType& client_id) override
    {
        std::string inference_solution = inference.to_string();
        std::transform(inference_solution.begin(), inference_solution.end(), inference_solution.begin(),
                    [](unsigned char c){ return std::tolower(c); }
                    );
        std::cout << "Data received from client: " << client_id
                                                    << " with id: " << task_id
                                                    << " job: " << inference.to_string()
                                                    << " inference: " << inference_solution << std::endl;
        return eprosima::amlip::types::InferenceSolutionDataType(inference_solution);
    }
};

class AsyncInferenceNodeJS : public eprosima::amlip::node::AsyncInferenceNode
{
public:

    AsyncInferenceNodeJS(
            const char* name,
            std::unique_ptr<InferenceReplierJS>& replier,
            uint32_t domain_id)
        : eprosima::amlip::node::AsyncInferenceNode(name, std::move(replier), domain_id)
    {
        // Do nothing
    }

    AsyncInferenceNodeJS(
            const char* name,
            std::unique_ptr<InferenceReplierJS>& replier)
        : eprosima::amlip::node::AsyncInferenceNode(name, std::move(replier))
    {
        // Do nothing
    }

};

%}
