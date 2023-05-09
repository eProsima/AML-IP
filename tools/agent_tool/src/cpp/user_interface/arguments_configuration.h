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
 * @file arg_configuration.h
 *
 */

#ifndef EPROSIMA_AGENT_TOOL_USERINTERFACE_ARG_CONFIGURATION_H_
#define EPROSIMA_AGENT_TOOL_USERINTERFACE_ARG_CONFIGURATION_H_

#include <iostream>
#include <string>

#include <optionparser.h>

constexpr const char* CLIENT_ENTITY_KIND_ARG = "client";
constexpr const char* SERVER_ENTITY_KIND_ARG = "server";
constexpr const char* REPEATER_ENTITY_KIND_ARG = "repeater";

struct Arg : public option::Arg
{
    static void print_error(
            const char* msg1,
            const option::Option& opt,
            const char* msg2)
    {
        fprintf(stderr, "%s", msg1);
        fwrite(opt.name, opt.namelen, 1, stderr);
        fprintf(stderr, "%s", msg2);
    }

    static option::ArgStatus Unknown(
            const option::Option& option,
            bool msg)
    {
        if (msg)
        {
            print_error("Unknown option '", option, "'\n");
        }
        return option::ARG_ILLEGAL;
    }

    static option::ArgStatus Required(
            const option::Option& option,
            bool msg)
    {
        if (option.arg != 0 && option.arg[0] != 0)
        {
            return option::ARG_OK;
        }

        if (msg)
        {
            print_error("Option '", option, "' requires an argument\n");
        }
        return option::ARG_ILLEGAL;
    }

    static option::ArgStatus Numeric(
            const option::Option& option,
            bool msg)
    {
        char* endptr = 0;
        if (option.arg != 0 && strtol(option.arg, &endptr, 10))
        {
        }
        if (endptr != option.arg && *endptr == 0)
        {
            return option::ARG_OK;
        }

        if (msg)
        {
            print_error("Option '", option, "' requires a numeric argument\n");
        }
        return option::ARG_ILLEGAL;
    }

    static option::ArgStatus String(
            const option::Option& option,
            bool msg)
    {
        if (option.arg != 0)
        {
            return option::ARG_OK;
        }
        if (msg)
        {
            print_error("Option '", option, "' requires a string argument\n");
        }
        return option::ARG_ILLEGAL;
    }

    static option::ArgStatus EntityKind(
            const option::Option& option,
            bool msg)
    {
        if (option.arg != 0)
        {
            std::string data_type = std::string(option.arg);
            if (data_type != CLIENT_ENTITY_KIND_ARG &&
                data_type != SERVER_ENTITY_KIND_ARG &&
                data_type != REPEATER_ENTITY_KIND_ARG)
            {
                if (msg)
                {
                    print_error("Option '", option, "' only accepts <client|server|repeater> values\n");
                }
                return option::ARG_ILLEGAL;
            }
            return option::ARG_OK;
        }
        if (msg)
        {
            print_error("Option '", option, "' requires a string argument\n");
        }
        return option::ARG_ILLEGAL;
    }

    static option::ArgStatus Transport(
            const option::Option& option,
            bool msg)
    {
        if (option.arg != 0)
        {
            std::string transport = std::string(option.arg);
            if (transport != "shm" && transport != "udp" && transport != "udpv4" && transport != "udpv6")
            {
                if (msg)
                {
                    print_error("Option '", option, "' only accepts <shm|udp[v4]|udpv6> values\n");
                }
                return option::ARG_ILLEGAL;
            }
            return option::ARG_OK;
        }
        if (msg)
        {
            print_error("Option '", option, "' requires a string argument\n");
        }
        return option::ARG_ILLEGAL;
    }
};

enum optionIndex
{
    UNKNOWN_OPT,
    ENTITY_TYPE,
    HELP,
    NAME,
    CONNECTION_PORT,
    LISTENING_PORT,
    DOMAIN_ID,
    CONNECTION_ADDRESS,
    LISTENING_ADDRESS,
    TRANSPORT
};

const option::Descriptor usage[] = {
    { UNKNOWN_OPT, 0, "", "",               Arg::None,
      "Usage: ./agent tool \n\nGeneral options:" },
    { HELP, 0, "h", "help",                 Arg::None,
      "  -h, --help  \tProduce help message." },
    { ENTITY_TYPE, 0, "e", "entity",       Arg::EntityKind,
      "  -e, --entity <client|server|repeater>  \tAgent Entity type (Default: client). Allowed options:\n \
                                    \t• client -> Run an Agent Client Node.\n \
                                    \t• server -> Run an Agent Server Node.\n \
                                    \t• repeater -> Run an Agent Repeater Node. " },
    { NAME, 0, "n", "name",          Arg::String,
      "  -n, --name <name>  \t Name (Default: agent_tool)." },
    { DOMAIN_ID, 0, "d", "domain",          Arg::Numeric,
      "  -d, --domain <id>  \tDDS domain ID (Default: 0)." },
    { CONNECTION_PORT, 0, "cp", "connection_port",          Arg::Numeric,
      "  -p, --connection-port <num>  \tAddress connection port (Default: 12121)." },
    { LISTENING_PORT, 0, "lp", "listening_port",          Arg::Numeric,
      "  -x, --listening-port <num>  \tAddress listening port (Default: 12121)." },
    { CONNECTION_ADDRESS, 0, "c", "connection_address",          Arg::String,
      "  -c, --connection <connection_address>  \tAddress to connect (Default: 127.0.0.1)." },
    { LISTENING_ADDRESS, 0, "l", "listening_address",           Arg::String,
      "  -l, --listening <listening_address>  \tAddress where listen (Default: 127.0.0.1)." },
    { TRANSPORT, 0, "t", "transport",         Arg::Transport,
      "  -t, --transport <shm|udp|udpv6>  \tUse only shared-memory, UDPv4, or UDPv6 transport. (Default: udp)." },
    { 0, 0, 0, 0, 0, 0 }
};

void print_warning(
        std::string type,
        const char* opt)
{
    std::cerr << "WARNING: " << opt << " is a " << type << " option, ignoring argument." << std::endl;
}

#endif /* EPROSIMA_AGENT_TOOL_USERINTERFACE_ARG_CONFIGURATION_H_ */
