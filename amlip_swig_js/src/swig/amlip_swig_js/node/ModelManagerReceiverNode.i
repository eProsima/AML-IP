// Copyright 2023 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
// Binding for class ModelManagerReceiverNode
////////////////////////////////////////////////////////

// Import parent class
%import(module="amlip_swig_js") "amlip_cpp/node/ParentNode.hpp";

%ignore eprosima::amlip::node::ModelListener;

%unique_ptr(ModelListenerJS);

%{
#include <amlip_cpp/node/collaborative_learning/ModelManagerReceiverNode.hpp>
%}

// Include the class interfaces
%include <amlip_cpp/node/collaborative_learning/ModelManagerReceiverNode.hpp>

%inline %{

bool block_modelmanagerreceiver_ = true;

class ModelListenerJS : public eprosima::amlip::node::ModelListener
{
public:

    ModelListenerJS()
    {
        // Do nothing
    }

    virtual ~ModelListenerJS()
    {
        // Do nothing
    }

    bool statistics_received(
            const eprosima::amlip::types::ModelStatisticsDataType statistics) override
    {
        std::cout << "Statistics received: " << statistics.to_string() << " ." << std::endl;
        // Decide if we want the model based on the statistics received
        return true;
    }

    bool model_received(
            const eprosima::amlip::types::ModelReplyDataType model) override
    {
        std::cout << "Reply received: " << model.to_string() << " ." << std::endl;
        block_modelmanagerreceiver_ = false;

        return true;
    }
};

class ModelManagerReceiverNodeJS : public eprosima::amlip::node::ModelManagerReceiverNode
{
public:

    ModelManagerReceiverNodeJS(
            eprosima::amlip::types::AmlipIdDataType id,
            eprosima::amlip::types::ModelRequestDataType data,
            uint32_t domain_id)
        : eprosima::amlip::node::ModelManagerReceiverNode(id, data, domain_id)
    {
        // Do nothing
    }

    ModelManagerReceiverNodeJS(
            eprosima::amlip::types::AmlipIdDataType id,
            eprosima::amlip::types::ModelRequestDataType data)
        : eprosima::amlip::node::ModelManagerReceiverNode(id, data)
    {
        // Do nothing
    }

    void start(
            std::unique_ptr<ModelListenerJS>& listener)
    {
        eprosima::amlip::node::ModelManagerReceiverNode::start(std::move(listener));
    }
};
%}
