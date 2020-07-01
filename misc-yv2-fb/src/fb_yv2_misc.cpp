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
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <gpiod.hpp>
#include <iostream>
#include <nlohmann/json.hpp>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <string_view>
#include <vector>

namespace fb_ipmi {
static boost::asio::io_service io;
std::shared_ptr<sdbusplus::asio::connection> conn;
static std::shared_ptr<sdbusplus::asio::dbus_interface> miscIface;

using respType =
    std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>;
static constexpr uint8_t lun = 0;
static constexpr uint8_t CPUPwrGdMask = 0x01;
static constexpr uint8_t PCHPwrGdMask = 0x02;

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

static bool
requestGPIOEvents(const std::string &name, const std::function<void()> &handler,
                  gpiod::line &gpioLine,
                  boost::asio::posix::stream_descriptor &gpioEventDescriptor) {
  // Find the GPIO line
  gpioLine = gpiod::find_line(name);
  if (!gpioLine) {
    std::cerr << "Failed to find the " << name << " line\n";
    return false;
  }

  try {
    gpioLine.request({"fb-yv2-misc", gpiod::line_request::EVENT_BOTH_EDGES});
  } catch (std::exception &) {
    std::cerr << "Failed to request events for " << name << "\n";
    return false;
  }

  int gpioLineFd = gpioLine.event_get_fd();
  if (gpioLineFd < 0) {
    std::cerr << "Failed to get " << name << " fd\n";
    return false;
  }
  gpioEventDescriptor.assign(gpioLineFd);

  gpioEventDescriptor.async_wait(
      boost::asio::posix::stream_descriptor::wait_read,
      [&name, handler](const boost::system::error_code ec) {
        if (ec) {
          std::cerr << name << " fd handler error: " << ec.message() << "\n";
          // TODO: throw here to force power-control to restart?
          return;
        }
        handler();
      });
  return true;
}

static void updateHandSwitchPosition() {
  std::cerr << "updateHandSwitchPosition started...\n";
  int position = 0x0;
  int line1 = HandSwitch1Line.get_value();
  int line2 = HandSwitch2Line.get_value();
  int line3 = HandSwitch3Line.get_value();
  int line4 = HandSwitch4Line.get_value();

  position = (position & 0xE) | (line1 << 0);
  position = (position & 0xD) | (line2 << 1);
  position = (position & 0xB) | (line3 << 2);
  position = (position & 0x7) | (line4 << 3);
  position += 1;

  miscIface->set_property("Position", position);
  std::cerr << "Position :" << position << "\n";
}

static void HandSwitch1Handler() {
  gpiod::line_event gpioLineEvent = HandSwitch1Line.event_read();

  if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE) {
    updateHandSwitchPosition();
  }
  if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE) {
    updateHandSwitchPosition();
  }
  HandSwitch1Event.async_wait(boost::asio::posix::stream_descriptor::wait_read,
                              [](const boost::system::error_code ec) {
                                if (ec) {
                                  std::cerr << "Hand Switch 1 handler error: "
                                            << ec.message() << "\n";
                                  return;
                                }
                                HandSwitch1Handler();
                              });
}

static void HandSwitch2Handler() {
  gpiod::line_event gpioLineEvent = HandSwitch2Line.event_read();

  if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE) {
    updateHandSwitchPosition();
  }
  if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE) {
    updateHandSwitchPosition();
  }
  HandSwitch2Event.async_wait(boost::asio::posix::stream_descriptor::wait_read,
                              [](const boost::system::error_code ec) {
                                if (ec) {
                                  std::cerr << "Hand Switch 2 handler error: "
                                            << ec.message() << "\n";
                                  return;
                                }
                                HandSwitch2Handler();
                              });
}

static void HandSwitch3Handler() {
  gpiod::line_event gpioLineEvent = HandSwitch3Line.event_read();

  if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE) {
    updateHandSwitchPosition();
  }
  if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE) {
    updateHandSwitchPosition();
  }
  HandSwitch3Event.async_wait(boost::asio::posix::stream_descriptor::wait_read,
                              [](const boost::system::error_code ec) {
                                if (ec) {
                                  std::cerr << "Hand Switch 3 handler error: "
                                            << ec.message() << "\n";
                                  return;
                                }
                                HandSwitch3Handler();
                              });
}

static void HandSwitch4Handler() {
  gpiod::line_event gpioLineEvent = HandSwitch4Line.event_read();

  if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE) {
    updateHandSwitchPosition();
  } else if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE) {
    updateHandSwitchPosition();
  }
  HandSwitch4Event.async_wait(boost::asio::posix::stream_descriptor::wait_read,
                              [](const boost::system::error_code ec) {
                                if (ec) {
                                  std::cerr << "Hand Switch 4 handler error: "
                                            << ec.message() << "\n";
                                  return;
                                }
                                HandSwitch4Handler();
                              });
}

static void powerButtonHandler() {
  gpiod::line_event gpioLineEvent = powerButtonLine.event_read();

  if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE) {
    std::cerr << "power button pressed = 1 \n";
    updateHandSwitchPosition();
    miscIface->set_property("PowerButtonPressed", 1);
  } else if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE) {
    std::cerr << "power button pressed = 0 \n";
    miscIface->set_property("PowerButtonPressed", 0);
  }
  powerButtonEvent.async_wait(boost::asio::posix::stream_descriptor::wait_read,
                              [](const boost::system::error_code ec) {
                                if (ec) {
                                  std::cerr << "power button handler error: "
                                            << ec.message() << "\n";
                                  return;
                                }
                                powerButtonHandler();
                              });
}

static void resetButtonHandler() {
  gpiod::line_event gpioLineEvent = resetButtonLine.event_read();

  if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE) {
    std::cerr << "Reset button pressed = 1 \n";
    updateHandSwitchPosition();
    miscIface->set_property("ResetButtonPressed", 1);
  } else if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE) {
    std::cerr << "Reset button pressed = 0 \n";
    miscIface->set_property("ResetButtonPressed", 0);
  }
  resetButtonEvent.async_wait(boost::asio::posix::stream_descriptor::wait_read,
                              [](const boost::system::error_code ec) {
                                if (ec) {
                                  std::cerr << "reset button handler error: "
                                            << ec.message() << "\n";
                                  return;
                                }
                                resetButtonHandler();
                              });
}

int sendIPMBRequest(uint8_t host, uint8_t netFn, uint8_t cmd,
                    std::vector<uint8_t> &cmdData,
                    std::vector<uint8_t> &respData) {

  auto method = conn->new_method_call("xyz.openbmc_project.Ipmi.Channel.Ipmb",
                                      "/xyz/openbmc_project/Ipmi/Channel/Ipmb",
                                      "org.openbmc.Ipmb", "sendRequest");
  method.append(host, netFn, lun, cmd, cmdData);

  auto reply = conn->call(method);
  if (reply.is_method_error()) {
    phosphor::logging::log<phosphor::logging::level::ERR>(
        "Error reading from IPMB");
    return -1;
  }

  respType resp;
  reply.read(resp);

  respData =
      std::move(std::get<std::remove_reference_t<decltype(respData)>>(resp));

  return 0;
}

static void setGpioConfiguration(uint8_t host) {
  int netFn = 0x38;
  int cmd = 0x6;
  std::vector<uint8_t> cmdData{
      0x15, 0xa0, 0x0, 0x03, 0x0, 0x0, 0x0, 0x0, 0x09,
  };
  std::vector<uint8_t> respData;

  sendIPMBRequest(host, netFn, cmd, cmdData, respData);
  int GpiosStatus = respData[0];
  std::cerr << "setGpioConfiguration Response: " << GpiosStatus << "\n"
            << std::flush;
}

static void BICInit() {
  int MAX_HOST = 2;
  for (int host = 0; host < MAX_HOST; host++) {
    setGpioConfiguration(host);
  }
}
}; // namespace fb_ipmi

int main(int argc, char *argv[]) {
  std::cerr << "Facebook Misc Ipmi service ....\n";

  fb_ipmi::conn = std::make_shared<sdbusplus::asio::connection>(fb_ipmi::io);

  fb_ipmi::conn->request_name("xyz.openbmc_project.Misc.Ipmi");

  // Request POWER_BUTTON GPIO events
  if (!fb_ipmi::requestGPIOEvents(
          "MULTI_HOST_POWER_BUTTON", fb_ipmi::powerButtonHandler,
          fb_ipmi::powerButtonLine, fb_ipmi::powerButtonEvent)) {
    return -1;
  }

  // Request RESET_BUTTON GPIO events
  if (!fb_ipmi::requestGPIOEvents(
          "MULTI_HOST_RESET_BUTTON", fb_ipmi::resetButtonHandler,
          fb_ipmi::resetButtonLine, fb_ipmi::resetButtonEvent)) {
    return -1;
  }

  // Request HAND_SW1 GPIO events
  if (!fb_ipmi::requestGPIOEvents("HAND_SW1", fb_ipmi::HandSwitch1Handler,
                                  fb_ipmi::HandSwitch1Line,
                                  fb_ipmi::HandSwitch1Event)) {
    return -1, true;
  }

  // Request HAND_SW1 GPIO events
  if (!fb_ipmi::requestGPIOEvents("HAND_SW2", fb_ipmi::HandSwitch2Handler,
                                  fb_ipmi::HandSwitch2Line,
                                  fb_ipmi::HandSwitch2Event)) {
    return -1;
  }

  // Request HAND_SW1 GPIO events
  if (!fb_ipmi::requestGPIOEvents("HAND_SW3", fb_ipmi::HandSwitch3Handler,
                                  fb_ipmi::HandSwitch3Line,
                                  fb_ipmi::HandSwitch3Event)) {
    return -1;
  }

  // Request HAND_SW1 GPIO events
  if (!fb_ipmi::requestGPIOEvents("HAND_SW4", fb_ipmi::HandSwitch4Handler,
                                  fb_ipmi::HandSwitch4Line,
                                  fb_ipmi::HandSwitch4Event)) {
    return -1;
  }

  fb_ipmi::BICInit();

  std::cerr << "After function powerGoodHandler \n";
  // Power Control Service
  sdbusplus::asio::object_server miscServer =
      sdbusplus::asio::object_server(fb_ipmi::conn);

  // Power Control Interface
  fb_ipmi::miscIface = miscServer.add_interface(
      "/xyz/openbmc_project/misc/ipmi", "xyz.openbmc_project.Misc.Ipmi");

  fb_ipmi::miscIface->register_property(
      "PowerButtonPressed", int(0),
      sdbusplus::asio::PropertyPermission::readWrite);
  fb_ipmi::miscIface->register_property(
      "ResetButtonPressed", int(0),
      sdbusplus::asio::PropertyPermission::readWrite);
  fb_ipmi::miscIface->register_property(
      "Power_Good1", int(0), sdbusplus::asio::PropertyPermission::readWrite);
  fb_ipmi::miscIface->register_property(
      "Power_Good2", int(0), sdbusplus::asio::PropertyPermission::readWrite);
  fb_ipmi::miscIface->register_property(
      "Position", int(0), sdbusplus::asio::PropertyPermission::readWrite);

  fb_ipmi::miscIface->initialize();

  fb_ipmi::io.run();

  return 0;
}
