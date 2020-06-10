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
static std::shared_ptr<sdbusplus::asio::dbus_interface> miscIface;

// GPIO Lines and Event Descriptors
static gpiod::line powerButtonLine;
static boost::asio::posix::stream_descriptor powerButtonEvent(io);
static gpiod::line resetButtonLine;
static boost::asio::posix::stream_descriptor resetButtonEvent(io);
static gpiod::line HandSwitch1Line;
static boost::asio::posix::stream_descriptor HandSwitch1Event(io);
static gpiod::line HandSwitch2Line;
static boost::asio::posix::stream_descriptor HandSwitch2Event(io);
static gpiod::line HandSwitch3Line;
static boost::asio::posix::stream_descriptor HandSwitch3Event(io);
static gpiod::line HandSwitch4Line;
static boost::asio::posix::stream_descriptor HandSwitch4Event(io);

static bool requestGPIOEvents(
    const std::string& name, const std::function<void()>& handler,
    gpiod::line& gpioLine,
    boost::asio::posix::stream_descriptor& gpioEventDescriptor)
{
        std::cerr << "requestGPIOEvents started 1\n";
    // Find the GPIO line
    gpioLine = gpiod::find_line(name);
    if (!gpioLine)
    {
        std::cerr << "Failed to find the " << name << " line\n";
        return false;
    }

    try
    {
        std::cerr << "requestGPIOEvents started 2\n";
        gpioLine.request(
            {"power-control", gpiod::line_request::EVENT_BOTH_EDGES});
    }
    catch (std::exception&)
    {
        std::cerr << "Failed to request events for " << name << "\n";
        return false;
    }

    int gpioLineFd = gpioLine.event_get_fd();
    if (gpioLineFd < 0)
    {
        std::cerr << "Failed to get " << name << " fd\n";
        return false;
    }
	gpioEventDescriptor.assign(gpioLineFd);

        std::cerr << "requestGPIOEvents started 3\n";
    gpioEventDescriptor.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [&name, handler](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << name << " fd handler error: " << ec.message()
                          << "\n";
                // TODO: throw here to force power-control to restart?
                return;
            }
        std::cerr << "requestGPIOEvents started 4\n";
            handler();
        });
    return true;
}

static void HandSwitch1Handler()
{
        std::cerr << "HandSwitch1Handler started\n";

    gpiod::line_event gpioLineEvent = HandSwitch1Line.event_read();

    if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE)
    {
        std::cerr << "Handler 1 FALLING_EDGE 1 \n";
        miscIface->set_property("HAND_SW1", 1);
    }
    else if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE)
    {
        std::cerr << "Handler 1 RISING _EDGE 0 \n";
   	    miscIface->set_property("HAND_SW1", 0);
    }
    HandSwitch1Event.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << "Hand Switch 1 handler error: " << ec.message()
                          << "\n";
                return;
            }
            HandSwitch1Handler();
        });
}

static void HandSwitch2Handler()
{
        std::cerr << "HandSwitch2Handler started\n";
    gpiod::line_event gpioLineEvent = HandSwitch1Line.event_read();

    if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE)
    {
        std::cerr << "Handler 2 FALLING_EDGE 1 \n";
        miscIface->set_property("HAND_SW2", "1");
    }
    else if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE)
    {
        std::cerr << "Handler 2 RISING _EDGE 0 \n";
        miscIface->set_property("HAND_SW2", "0");
    }
    HandSwitch2Event.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << "Hand Switch 2 handler error: " << ec.message()
                          << "\n";
                return;
            }
            HandSwitch2Handler();
        });
}

static void HandSwitch3Handler()
{
        std::cerr << "HandSwitch3Handler started\n";
    gpiod::line_event gpioLineEvent = HandSwitch1Line.event_read();

    if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE)
    {
        miscIface->set_property("HAND_SW3", "1");
    }
    else if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE)
    {
        miscIface->set_property("HAND_SW3", "0");
    }
    HandSwitch3Event.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << "Hand Switch 3 handler error: " << ec.message()
                          << "\n";
                return;
            }
            HandSwitch3Handler();
        });
}

static void HandSwitch4Handler()
{
        std::cerr << "HandSwitch4Handler started\n";
    gpiod::line_event gpioLineEvent = HandSwitch1Line.event_read();

    if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE)
    {
        miscIface->set_property("HAND_SW4", "1");
    }
    else if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE)
    {
        miscIface->set_property("HAND_SW4", "0");
    }
    HandSwitch4Event.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << "Hand Switch 4 handler error: " << ec.message()
                          << "\n";
                return;
            }
            HandSwitch4Handler();
        });
}

/*static void findServerSlotSelect()
{
	auto sw1 =  miscIface->get_property("HAND_SW4");
	std::cerr << "Switch One Value: " << sw1 << "\n";
}*/

static void powerButtonHandler()
{
		std::cerr << "power button Handler \n"; 
	gpiod::line_event gpioLineEvent = powerButtonLine.event_read();

	if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE)
	{
		std::cerr << "power button pressed \n"; 
		miscIface->set_property("PowerButtonPressed", true);
	}
	else if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE)
	{
		miscIface->set_property("PowerButtonPressed", false);
	}
	powerButtonEvent.async_wait(
		boost::asio::posix::stream_descriptor::wait_read,
		[](const boost::system::error_code ec) {
			if (ec)
			{
				std::cerr << "power button handler error: " << ec.message()
						  << "\n";
				return;
			}
			powerButtonHandler();
		});
}


static void resetButtonHandler()
{
		std::cerr << "reset button Handler \n"; 
    gpiod::line_event gpioLineEvent = resetButtonLine.event_read();

    if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE)
    {
		std::cerr << "Reset button pressed \n"; 
        miscIface->set_property("ResetButtonPressed", true);
    }
    else if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE)
    {
        miscIface->set_property("ResetButtonPressed", false);
    }
    resetButtonEvent.async_wait(
        boost::asio::posix::stream_descriptor::wait_read,
        [](const boost::system::error_code ec) {
            if (ec)
            {
                std::cerr << "reset button handler error: " << ec.message()
                          << "\n";
                return;
            }
            resetButtonHandler();
        });
}
};

int main(int argc, char* argv[])
{
    std::cerr <<  "Facebook Misc Ipmi service ...\n";
	
	fb_ipmi::conn =
        std::make_shared<sdbusplus::asio::connection>(fb_ipmi::io);
	
	fb_ipmi::conn->request_name("xyz.openbmc_project.Misc.Ipmi");


	// Request POWER_BUTTON GPIO events
    if (!fb_ipmi::requestGPIOEvents(
            "POWER_BUTTON", fb_ipmi::powerButtonHandler,
            fb_ipmi::powerButtonLine, fb_ipmi::powerButtonEvent))
    {
        return -1;
    }

    // Request RESET_BUTTON GPIO events
    if (!fb_ipmi::requestGPIOEvents(
            "RESET_BUTTON", fb_ipmi::resetButtonHandler,
            fb_ipmi::resetButtonLine, fb_ipmi::resetButtonEvent))
    {
        return -1;
    }	

    // Request HAND_SW1 GPIO events
    if (!fb_ipmi::requestGPIOEvents(
            "HAND_SW1", fb_ipmi::HandSwitch1Handler,
            fb_ipmi::HandSwitch1Line, fb_ipmi::HandSwitch1Event))
    {
        return -1;
    }	

    // Request HAND_SW1 GPIO events
    if (!fb_ipmi::requestGPIOEvents(
            "HAND_SW2", fb_ipmi::HandSwitch2Handler,
            fb_ipmi::HandSwitch2Line, fb_ipmi::HandSwitch2Event))
    {
        return -1;
    }	

    // Request HAND_SW1 GPIO events
    if (!fb_ipmi::requestGPIOEvents(
            "HAND_SW3", fb_ipmi::HandSwitch3Handler,
            fb_ipmi::HandSwitch3Line, fb_ipmi::HandSwitch3Event))
    {
        return -1;
    }	

    // Request HAND_SW1 GPIO events
    if (!fb_ipmi::requestGPIOEvents(
            "HAND_SW4", fb_ipmi::HandSwitch4Handler,
            fb_ipmi::HandSwitch4Line, fb_ipmi::HandSwitch4Event))
    {
        return -1;
    }	

	// Power Control Service
    sdbusplus::asio::object_server miscServer =
        sdbusplus::asio::object_server(fb_ipmi::conn);

    // Power Control Interface
    fb_ipmi::miscIface = miscServer.add_interface(
        "/xyz/openbmc_project/misc/ipmi", "xyz.openbmc_project.Misc.Ipmi");

	fb_ipmi::miscIface->register_property("PowerButtonPressed",
                                          false,sdbusplus::asio::PropertyPermission::readWrite);
	fb_ipmi::miscIface->register_property("ResetButtonPressed",
                                          false,sdbusplus::asio::PropertyPermission::readWrite);
    fb_ipmi::miscIface->register_property(
        "HAND_SW1", int(0),sdbusplus::asio::PropertyPermission::readWrite);
    fb_ipmi::miscIface->register_property(
        "HAND_SW2", int(0));
    fb_ipmi::miscIface->register_property(
        "HAND_SW3", int(0));
    fb_ipmi::miscIface->register_property(
        "HAND_SW4", int(0));
    fb_ipmi::miscIface->register_signal<int>("position");

    fb_ipmi::miscIface->initialize();

/*
	auto method = fb_ipmi::conn->new_method_call("xyz.openbmc_project.Misc.Ipmi", 
								  "/xyz/openbmc_project/misc/ipmi",
						          "org.freedesktop.DBus.Properties", "Get");
	method.append("xyz.openbmc_project.Misc.Ipmi","HAND_SW1");
	
            std::cerr<< "Working till here 1 \n";
	auto reply = fb_ipmi::conn->call(method);
	if (reply.is_method_error())
    {
            std::cerr<< "Error in reading \n";
        return -1;
    }
            std::cerr<< "Working till here 2 \n";
	std::vector<uint8_t> resp;
	reply.read(resp);
            std::cerr<< "Working till here 3 \n";
	
//	std::cerr <<" The Value : "<< resp << "\n";

*/
	//fb_ipmi::findServerSlotSelect();

	fb_ipmi::io.run();
	//fb_ipmi::findServerSlotSelect();

	return 0;
}

