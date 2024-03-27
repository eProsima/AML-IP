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
// Binding for class InferenceNode
////////////////////////////////////////////////////////

// Import parent class
%import(module="amlip_swig_js") "amlip_cpp/node/ParentNode.hpp";

%ignore eprosima::amlip::node::InferenceListener;

%{
#include <amlip_cpp/node/InferenceNode.hpp>
%}

// Include the class interfaces
%include <amlip_cpp/node/InferenceNode.hpp>

%inline %{

class InferenceListenerJS : public eprosima::amlip::node::InferenceListener
{
public:

    InferenceListenerJS()
    {
        // Do nothing
    }

    virtual ~InferenceListenerJS()
    {
        // Do nothing
    }

    eprosima::amlip::types::InferenceSolutionDataType process_inference(const eprosima::amlip::types::InferenceDataType& inference) const override
    {
        std::string inference_solution = inference.to_string();
        std::transform(inference_solution.begin(), inference_solution.end(), inference_solution.begin(),
                    [](unsigned char c){ return std::tolower(c); }
                    );
        return eprosima::amlip::types::InferenceSolutionDataType(inference_solution);
    }
};

%}
