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
// Binding for class ComputingNode
////////////////////////////////////////////////////////

// Import parent class
%import(module="amlip_swig_js") "amlip_cpp/node/ParentNode.hpp";

%ignore eprosima::amlip::node::JobListener;

%{
#include <amlip_cpp/node/workload_distribution/ComputingNode.hpp>
%}

// Include the class interfaces
%include <amlip_cpp/node/workload_distribution/ComputingNode.hpp>

%inline %{

class JobListenerJS : public eprosima::amlip::node::JobListener
{
public:
    JobListenerJS()
    {
        // Do nothing
    }

    virtual ~JobListenerJS()
    {
        // Do nothing
    }

    virtual eprosima::amlip::types::JobSolutionDataType process_job(const eprosima::amlip::types::JobDataType& job) const
    {
        std::string job_solution = job.to_string();
        std::transform(job_solution.begin(), job_solution.end(), job_solution.begin(),
                    [](unsigned char c){ return std::tolower(c); }
                    );
        return eprosima::amlip::types::JobSolutionDataType(job_solution);
    }
};

%}
