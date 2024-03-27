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
// Binding for class ModelManagerSenderNode
////////////////////////////////////////////////////////

// Import parent class
%import(module="amlip_swig_js") "amlip_cpp/node/ParentNode.hpp";

%ignore eprosima::amlip::node::ModelReplier;

%unique_ptr(ModelReplierJS)

%{
#include <amlip_cpp/node/collaborative_learning/ModelManagerSenderNode.hpp>
%}

// Include the class interfaces
%include <amlip_cpp/node/collaborative_learning/ModelManagerSenderNode.hpp>

%inline %{

bool block_modelmanagersender_ = true;

class ModelReplierJS : public eprosima::amlip::node::ModelReplier
{
public:

    ModelReplierJS()
    {
        // Do nothing
    }

    virtual ~ModelReplierJS()
    {
        // Do nothing
    }

    eprosima::amlip::types::ModelReplyDataType fetch_model(
            const eprosima::amlip::types::ModelRequestDataType request) override
    {
        std::cout << "Request received: " << request.to_string() << std::endl;
        std::string reply = request.to_string();
        std::transform(reply.begin(), reply.end(), reply.begin(),
                    [](unsigned char c){ return std::tolower(c); }
                    );
        block_modelmanagersender_ = false;

        return eprosima::amlip::types::ModelReplyDataType(reply);
    }
};

class ModelManagerSenderNodeJS : public eprosima::amlip::node::ModelManagerSenderNode
{
public:

    ModelManagerSenderNodeJS(
            eprosima::amlip::types::AmlipIdDataType id,
            uint32_t domain_id)
        : eprosima::amlip::node::ModelManagerSenderNode(id, domain_id)
    {
        // Do nothing
    }

    ModelManagerSenderNodeJS(
            eprosima::amlip::types::AmlipIdDataType id)
        : eprosima::amlip::node::ModelManagerSenderNode(id)
    {
        // Do nothing
    }

    void start(
            std::unique_ptr<ModelReplierJS>& replier)
    {
        eprosima::amlip::node::ModelManagerSenderNode::start(std::move(replier));
    }
};
%}
