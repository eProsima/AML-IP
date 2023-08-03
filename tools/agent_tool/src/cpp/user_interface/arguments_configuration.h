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

#ifndef EPROSIMA_AMLIP_AGENT_USERINTERFACE_ARG_CONFIGURATION_H_
#define EPROSIMA_AMLIP_AGENT_USERINTERFACE_ARG_CONFIGURATION_H_

#include <iostream>
#include <string>

#include <cpp_utils/Log.hpp>
#include <cpp_utils/utils.hpp>
#include <cpp_utils/macros/custom_enumeration.hpp>

#include <optionparser.h>

constexpr const char* CLIENT_ENTITY_KIND_ARG = "client";
constexpr const char* SERVER_ENTITY_KIND_ARG = "server";
constexpr const char* REPEATER_ENTITY_KIND_ARG = "repeater";

ENUMERATION_BUILDER(
    LogKind,
    error,
    warning,
    info
    );

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
            if (transport != "tcp" && transport != "udp")
            {
                if (msg)
                {
                    print_error("Option '", option, "' only accepts <tcp|udp> values\n");
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

    static option::ArgStatus ValidOptions(
            const std::vector<std::string>& valid_options,
            const option::Option& option,
            bool msg)
    {
        if (nullptr == option.arg)
        {
            if (msg)
            {
                print_error("Option '", option, "' requires a text argument.");
            }
            return option::ARG_ILLEGAL;
        }

        if (std::find(valid_options.begin(), valid_options.end(), std::string(option.arg)) != valid_options.end())
        {
            return option::ARG_OK;
        }
        else if (msg)
        {
            eprosima::utils::Formatter error_msg;
            error_msg << "Option '" << option << "' requires a one of this values: {";
            for (const auto& valid_option : valid_options)
            {
                error_msg << "\"" << valid_option << "\";";
            }
            error_msg << "}.";

            logError(AMLIP_AGENT_TOOL_ARGS, error_msg);
        }

        return option::ARG_ILLEGAL;
    }

    static option::ArgStatus Log_Kind_Correct_Argument(
            const option::Option& option,
            bool msg)
    {
        return ValidOptions(string_vector_LogKind(), option, msg);
    }

};

enum optionIndex
{
    UNKNOWN_OPT,
    ENTITY_TYPE,
    HELP,
    ACTIVATE_DEBUG,
    LOG_FILTER,
    LOG_VERBOSITY,
    NAME,
    CONNECTION_PORT,
    LISTENING_PORT,
    DOMAIN_ID,
    CONNECTION_ADDRESS,
    LISTENING_ADDRESS,
    TRANSPORT
};

const option::Descriptor usage[] = {
    { UNKNOWN_OPT, 0, "", "", Arg::None, "\nUsage: ./agent tool \n\nGeneral options:" },
    { HELP, 0, "h", "help", Arg::None,
      "  -h, --help \tProduce help message." },
    { ENTITY_TYPE, 0, "e", "entity", Arg::EntityKind,
      "  -e, --entity <client|server|repeater> \tAgent Entity type (Default: client). \n" \
      " \tAllowed options: \n" \
      " \t• client -> Run an Agent Client Node.\n" \
      " \t• server -> Run an Agent Server Node.\n" \
      " \t• repeater -> Run an Agent Repeater Node. "},

    /// DEBUG OPTIONS
    { UNKNOWN_OPT, 0, "", "", Arg::None, "\nDebug options:"},
    { ACTIVATE_DEBUG, 0, "d", "debug", Arg::None,
      "  -d, --debug \tSet log verbosity to Info (Using this option with --log-filter and/or --log-verbosity " \
      "will head to undefined behaviour)." },
    { LOG_FILTER, 0, "", "log-filter", Arg::String,
      "      --log-filter \tSet a Regex Filter to filter by category the info and warning log entries. " \
      "(Default = AMLIP)." },
    { LOG_VERBOSITY, 0, "", "log-verbosity", Arg::Log_Kind_Correct_Argument,
      "      --log-verbosity <info|warning|error> \tSet a Log Verbosity Level higher or equal the one given " \
      "(Default: warning). " },

    /// CLIENT OPTIONS
    { UNKNOWN_OPT, 0, "", "", Arg::None, "\nClient options:"},
    { NAME, 0, "n", "name", Arg::String,
      "  -n, --name <name> \tName (Default: amlip_agent)." },
    { DOMAIN_ID, 0, "d", "domain", Arg::Numeric,
      "  -d, --domain <id> \tDDS domain ID (Default: 0)." },
    { CONNECTION_ADDRESS, 0, "c", "connection-address", Arg::String,
      "  -c, --connection-address <address> \tAddress to connect (Default: 127.0.0.1)." },
    { CONNECTION_PORT, 0, "p", "connection-port", Arg::Numeric,
      "  -p, --connection-port <num> \tAddress connection port (Default: 12121)." },
    { TRANSPORT, 0, "t", "transport", Arg::Transport,
      "  -t, --transport <tcp|udp> \tUse only TCPv4 or UDPv4 transport. (Default: TCPv4)." },

    /// SERVER OPTIONS
    { UNKNOWN_OPT, 0, "", "", Arg::None, "\nServer options:"},
    { NAME, 0, "n", "name", Arg::String,
      "  -n, --name <name> \tName (Default: amlip_agent)." },
    { DOMAIN_ID, 0, "d", "domain", Arg::Numeric,
      "  -d, --domain <id> \tDDS domain ID (Default: 0)." },
    { LISTENING_ADDRESS, 0, "l", "listening-address", Arg::String,
      "  -l, --listening-address <address> \tAddress where listen (Default: 127.0.0.1)." },
    { LISTENING_PORT, 0, "q", "listening-port", Arg::Numeric,
      "  -q, --listening-port <num> \tAddress listening port (Default: 12121)." },
    { TRANSPORT, 0, "t", "transport", Arg::Transport,
      "  -t, --transport <tcp|udp> \tUse only TCPv4 or UDPv4 transport. (Default: TCPv4)." },

    /// REPEATER OPTIONS
    { UNKNOWN_OPT, 0, "", "", Arg::None, "\nRepeater options:"},
    { NAME, 0, "n", "name", Arg::String,
      "  -n, --name <name> \tName (Default: amlip_agent)." },
    { DOMAIN_ID, 0, "d", "domain", Arg::Numeric,
      "  -d, --domain <id> \tDDS domain ID (Default: 0)." },
    { CONNECTION_ADDRESS, 0, "c", "connection-address", Arg::String,
      "  -c, --connection-address <address> \tAddress to connect (Default: 127.0.0.1)." },
    { LISTENING_ADDRESS, 0, "l", "listening-address", Arg::String,
      "  -l, --listening-address <address> \tAddress where listen (Default: 127.0.0.1)." },
    { CONNECTION_PORT, 0, "p", "connection-port", Arg::Numeric,
      "  -p, --connection-port <num> \tAddress connection port (Default: 12121)." },
    { LISTENING_PORT, 0, "q", "listening-port", Arg::Numeric,
      "  -q, --listening-port <num> \tAddress listening port (Default: 12121)." },
    { TRANSPORT, 0, "t", "transport", Arg::Transport,
      "  -t, --transport <tcp|udp> \tUse only TCPv4 or UDPv4 transport. (Default: TCPv4)." },

    { UNKNOWN_OPT, 0, "", "", Arg::None, " \t\t\t"},
    { 0, 0, 0, 0, 0, 0 }
};

void print_warning(
        std::string type,
        const char* opt)
{
    std::cerr << "WARNING: " << opt << " is a " << type << " option, ignoring argument." << std::endl;
}

std::ostream& operator <<(
        std::ostream& output,
        const option::Option& option)
{
    output << std::string(option.name, option.name + option.namelen);
    return output;
}

#endif /* EPROSIMA_AMLIP_AGENT_USERINTERFACE_ARG_CONFIGURATION_H_ */
