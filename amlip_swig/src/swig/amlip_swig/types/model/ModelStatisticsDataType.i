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
// Binding for class ModelStatisticsDataType
////////////////////////////////////////////////////////

// Import parent class
%import(module="amlip_swig") "amlip_cpp/types/InterfaceDataType.hpp";

// Ignore overloaded methods that have no application on Python
// Otherwise they will issue a warning
%ignore eprosima::amlip::types::ModelStatisticsDataType::ModelStatisticsDataType(ModelStatisticsDataType&&);
%ignore eprosima::amlip::types::ModelStatisticsDataType::name();
%rename("%s") eprosima::amlip::types::ModelStatisticsDataType::name() const;
%ignore eprosima::amlip::types::ModelStatisticsDataType::data_size();
%rename("%s") eprosima::amlip::types::ModelStatisticsDataType::data_size() const;
%ignore eprosima::amlip::types::ModelStatisticsDataType::server_id();
%rename("%s") eprosima::amlip::types::ModelStatisticsDataType::server_id() const;

// Declare the to string method
%extend eprosima::amlip::types::ModelStatisticsDataType {
    std::string __str__() const
    {
        return $self->to_string();
    }
}

%{
#include <amlip_cpp/types/model/ModelStatisticsDataType.hpp>

%}

// Include the class interfaces
%include <amlip_cpp/types/model/ModelStatisticsDataType.hpp>
