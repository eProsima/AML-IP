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
// Binding for class AsyncComputingNode
////////////////////////////////////////////////////////

// Import parent class
%import(module="amlip_swig_js") "amlip_cpp/node/ParentNode.hpp";

%ignore eprosima::amlip::node::JobReplier;

%unique_ptr(JobReplierJS);

%{
#include <amlip_cpp/node/workload_distribution/AsyncComputingNode.hpp>
%}

// Include the class interfaces
%include <amlip_cpp/node/workload_distribution/AsyncComputingNode.hpp>

%inline %{

class JobReplierJS : public eprosima::amlip::node::JobReplier
{
public:

    JobReplierJS()
    {
        // Do nothing
    }

    virtual ~JobReplierJS()
    {
        // Do nothing
    }

    virtual eprosima::amlip::types::JobSolutionDataType process_job(
            const eprosima::amlip::types::JobDataType& job,
            const eprosima::amlip::types::TaskId& task_id,
            const eprosima::amlip::types::AmlipIdDataType& client_id)
    {
        std::string job_solution = job.to_string();
        std::transform(job_solution.begin(), job_solution.end(), job_solution.begin(),
                    [](unsigned char c){ return std::tolower(c); }
                    );
        std::cout << "Data received from client: " << client_id
                                                    << " with id: " << task_id
                                                    << " job: " << job.to_string()
                                                    << " inference: " << job_solution << std::endl;
        return eprosima::amlip::types::JobSolutionDataType(job_solution);
    }
};

class AsyncComputingNodeJS : public eprosima::amlip::node::AsyncComputingNode
{
public:

    AsyncComputingNodeJS(
            const char* name,
            std::unique_ptr<JobReplierJS>& replier,
            uint32_t domain_id)
        : eprosima::amlip::node::AsyncComputingNode(name, std::move(replier), domain_id)
    {
        // Do nothing
    }

    AsyncComputingNodeJS(
            const char* name,
            std::unique_ptr<JobReplierJS>& replier)
        : eprosima::amlip::node::AsyncComputingNode(name, std::move(replier))
    {
        // Do nothing
    }
};

%}
