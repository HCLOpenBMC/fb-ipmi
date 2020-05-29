/*
/ Copyright (c) 2019-2020 Facebook Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include <sys/sysinfo.h>
#include <systemd/sd-journal.h>

#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <filesystem>
#include <fstream>
#include <gpiod.hpp>
#include <iostream>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <string_view>
#include <vector>
#include <nlohmann/json.hpp>

namespace fb_ipmi
{
	static boost::asio::io_service io;
	std::shared_ptr<sdbusplus::asio::connection> conn;
	static std::shared_ptr<sdbusplus::asio::dbus_interface> hostIface;
	std::string node = "0";
	
};

int main(int argc, char* argv[])
{
    std::cerr <<  "dharshan Facebook Ipmi service ...\n";
	
	fb_ipmi::conn =
        std::make_shared<sdbusplus::asio::connection>(fb_ipmi::io);
	
	fb_ipmi::conn->request_name("xyz.openbmc_project.State.Ipmi");
	

	// Power Control Service
    sdbusplus::asio::object_server hostServer =
        sdbusplus::asio::object_server(fb_ipmi::conn);

    // Power Control Interface
    fb_ipmi::hostIface = hostServer.add_interface(
        "/xyz/openbmc_project/state/ipmi", "xyz.openbmc_project.State.Ipmi");

    fb_ipmi::hostIface->register_property(
        "RequestedHostTransition",
        std::string("xyz.openbmc_project.State.Ipmi.Transition.Off"),
        [](const std::string& requested, std::string& resp) {
            if (requested == "xyz.openbmc_project.State.Ipmi.Transition.Off")
            {
			std::cerr << "Host" << fb_ipmi::node << ": " <<  "Off Requeset...\n";
            }
            else if (requested ==
                     "xyz.openbmc_project.State.Ipmi.Transition.On")
            {
				std::cerr << "Host" << fb_ipmi::node << ": " <<  "Off Requeset...\n";
            }
            else 
            {
			std::cerr << "Host" << fb_ipmi::node << ": " <<  "Unrecognized host state transition request.\n";
                throw std::invalid_argument("Unrecognized Transition Request");
                return 0;
            }
            resp = requested;
            return 1;
        });
    fb_ipmi::hostIface->initialize();

	fb_ipmi::io.run();


	return 0;

}

