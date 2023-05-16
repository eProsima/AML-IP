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

/**
 * @file agent_tool.cpp
 *
 */

#include <algorithm>
#include <thread>

#include <cpp_utils/event/SignalEventHandler.hpp>
#include <cpp_utils/Log.hpp>
#include <ddsrouter_core/types/address/Address.hpp>

#include <amlip_cpp/node/wan/ClientNode.hpp>
#include <amlip_cpp/node/wan/ServerNode.hpp>
#include <amlip_cpp/node/wan/TurnNode.hpp>

#include "user_interface/arguments_configuration.h"

enum EntityType
{
    CLIENT,
    SERVER,
    REPEATER
};

int main(
        int argc,
        char** argv)
{
    eprosima::utils::event::SignalEventHandler<eprosima::utils::event::Signal::sigint> sigint_handler;

    // Help message formatting settings
    int columns;

#if defined(_WIN32)
    char* buf = nullptr;
    size_t sz = 0;
    if (_dupenv_s(&buf, &sz, "COLUMNS") == 0 && buf != nullptr)
    {
        columns = strtol(buf, nullptr, 10);
        free(buf);
    }
    else
    {
        columns = 80;
    }
#else
    columns = getenv("COLUMNS") ? atoi(getenv("COLUMNS")) : 80;
#endif // if defined(_WIN32)

    // Parameter definition
    EntityType entity_type = EntityType::CLIENT;
    std::string ip = "127.0.0.1";
    int domain = 0;
    int connection_port = 12121;
    int listening_port = 12121;
    int port = 12121;
    std::string name = "agent_tool";
    eprosima::ddsrouter::core::types::TransportProtocol transport_protocol =
            eprosima::ddsrouter::core::types::TransportProtocol::udp;

    // Parse example options
    argc -= (argc > 0);
    argv += (argc > 0); // skip program name argv[0] if present
    option::Stats stats(usage, argc, argv);
    std::vector<option::Option> options(stats.options_max);
    std::vector<option::Option> buffer(stats.buffer_max);
    option::Parser parse(usage, argc, argv, &options[0], &buffer[0]);

    if (parse.error())
    {
        option::printUsage(fwrite, stdout, usage, columns);
        return 1;
    }

    if (options[optionIndex::HELP])
    {
        option::printUsage(fwrite, stdout, usage, columns);
        return 0;
    }

    for (int i = 0; i < parse.optionsCount(); ++i)
    {
        option::Option& opt = buffer[i];
        switch (opt.index())
        {
            case optionIndex::HELP:
                // Not possible as this argument has been already handled and it exits the program
                break;

            case optionIndex::ENTITY_TYPE:
                if (strcmp(opt.arg, CLIENT_ENTITY_KIND_ARG) == 0)
                {
                    entity_type = EntityType::CLIENT;
                }
                else if (strcmp(opt.arg, SERVER_ENTITY_KIND_ARG) == 0)
                {
                    entity_type = EntityType::SERVER;
                }
                else if (strcmp(opt.arg, REPEATER_ENTITY_KIND_ARG) == 0)
                {
                    entity_type = EntityType::REPEATER;
                }
                else
                {
                    std::cerr << "ERROR: incorrect entity type. Only <client|server|repeater> accepted." << std::endl;
                    return 1;
                }
                break;

            case optionIndex::CONNECTION_ADDRESS:
                ip = std::string(opt.arg);
                break;

            case optionIndex::LISTENING_ADDRESS:
                ip = std::string(opt.arg);
                break;

            case optionIndex::NAME:
                name = std::string(opt.arg);
                break;

            case optionIndex::CONNECTION_PORT:
                connection_port = strtol(opt.arg, nullptr, 10);
                break;

            case optionIndex::LISTENING_PORT:
                listening_port = strtol(opt.arg, nullptr, 10);
                break;

            case optionIndex::DOMAIN_ID:
                domain = strtol(opt.arg, nullptr, 10);
                break;

            case optionIndex::TRANSPORT:
                if (strcmp(opt.arg, "tcp") == 0)
                {
                    transport_protocol = eprosima::ddsrouter::core::types::TransportProtocol::tcp;
                }
                else if (strcmp(opt.arg, "udp") == 0)
                {
                    transport_protocol = eprosima::ddsrouter::core::types::TransportProtocol::udp;
                }
                break;

            case optionIndex::UNKNOWN_OPT:
                std::cerr << "ERROR: " << opt.name << " is not a valid argument." << std::endl;
                option::printUsage(fwrite, stdout, usage, columns);
                return 1;
                break;
        }
    }

    {
        if (entity_type == CLIENT)
        {
            port = connection_port;
            logUser(AMLIPCPP_MANUAL_TEST, "Starting Manual Agent Node execution. Creating connection address...");

        }
        else
        {
            port = listening_port;
            logUser(AMLIPCPP_MANUAL_TEST, "Starting Manual Agent Node execution. Creating listening address...");
        }

        // Create connection address
        auto address = eprosima::ddsrouter::core::types::Address(
            ip,
            port,
            port,
            transport_protocol);

        std::set<eprosima::ddsrouter::core::types::Address> addresses = { address };

        std::shared_ptr<eprosima::amlip::node::agent::AgentNode> agent_node;

        try
        {
            switch (entity_type)
            {
                case CLIENT:
                {
                    logUser(AMLIPCPP_MANUAL_TEST, "Address to connect: " << address << ". Creating Node...");

                    // Create Client Node
                    agent_node = std::make_shared<eprosima::amlip::node::agent::ClientNode>(
                        name.c_str(),
                        addresses,
                        domain);
                    break;
                }
                case SERVER:
                {
                    logUser(AMLIPCPP_MANUAL_TEST, "Address where listen: " << address << ". Creating Node...");

                    // Create Server Node
                    agent_node = std::make_shared<eprosima::amlip::node::agent::ServerNode>(
                        name.c_str(),
                        addresses,
                        domain);
                    break;
                }
                case REPEATER:
                {
                    logUser(AMLIPCPP_MANUAL_TEST, "Address where listen: " << address << ". Creating Node...");

                    // Create Turn Node
                    agent_node = std::make_shared<eprosima::amlip::node::agent::TurnNode>(
                        name.c_str(),
                        addresses);
                    break;
                }
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "Execution failed with error:\n " << e.what() << std::endl;
        }

        logUser(AMLIPCPP_MANUAL_TEST, "Node created. Waiting SIGINT (C^) to close...");

        sigint_handler.wait_for_event();

        logUser(AMLIPCPP_MANUAL_TEST, "SIGINT received. Destroying entities...");

    }

    logUser(AMLIPCPP_MANUAL_TEST, "Finishing Agent Node execution.");

    return 0;
}
