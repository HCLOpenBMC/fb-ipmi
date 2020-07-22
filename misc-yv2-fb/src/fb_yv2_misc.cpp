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
      0x15, 0xa0, 0x0, 0x03, 0x0, 0x0, 0x0, 0x0, 0x0, 0x12, 0x12
  };
  std::vector<uint8_t> respData;

  sendIPMBRequest(host, netFn, cmd, cmdData, respData);

    std::cerr << "SetGpioConfiguration Response:\n" << std::flush;
    for(int i=0; i<respData.size(); i++)
    {
        printf("%x:", respData[i]);
    }
    std::cout.flush();  
}

static void getGpioConfiguration(uint8_t host) {
  int netFn = 0x38;
  int cmd = 0x5;
  std::vector<uint8_t> cmdData{
      0x15, 0xa0, 0x0, 0x03, 0x0, 0x0, 0x0, 0x0,
  };
  std::vector<uint8_t> respData;

  sendIPMBRequest(host, netFn, cmd, cmdData, respData);
    std::cerr << "GetGpioConfiguration Response:\n" << std::flush;
    for(int i=0; i<respData.size(); i++)
    {
        printf("%x:", respData[i]);
    }
    std::cout.flush();
}
#define CMD_OEM_1S_GET_CONFIG 0x0E
#define CMD_OEM_1S_SET_CONFIG 0x10
#define NETFN_OEM_1S_REQ 0x38
#define CMD_OEM_1S_GET_POST_BUF 0x08

#define GPIO_VAL "/sys/class/gpio/gpio%d/value"
#define GPIO_BASE_NUM 280
#define GPIO_POSTCODE_0 (48 + GPIO_BASE_NUM)
#define GPIO_POSTCODE_1 (49 + GPIO_BASE_NUM)
#define GPIO_POSTCODE_2 (50 + GPIO_BASE_NUM)
#define GPIO_POSTCODE_3 (51 + GPIO_BASE_NUM)
#define GPIO_POSTCODE_4 (124 + GPIO_BASE_NUM)
#define GPIO_POSTCODE_5 (125 + GPIO_BASE_NUM)
#define GPIO_POSTCODE_6 (126 + GPIO_BASE_NUM)
#define GPIO_POSTCODE_7 (127 + GPIO_BASE_NUM)

#define BIT(value, index) ((value >> index) & 1)

#define GPIO_DBG_CARD_PRSNT (139 + GPIO_BASE_NUM)

static std::shared_ptr<sdbusplus::asio::dbus_interface> miscPface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> miscP1face;
static std::shared_ptr<sdbusplus::asio::dbus_interface> miscP2face;


static constexpr bool DEBUG = false;



typedef struct _bic_config_t {
  union {
    struct {
      uint8_t sol : 1;
      uint8_t post : 1;
      uint8_t rsvd : 6;
    };
    uint8_t config;
  };
} bic_config_t;


static int write_device(const char *device, const char *value) {
  FILE *fp;
  int rc;

  if (DEBUG)
  std::cout << "write_device()" << std::endl;

  fp = fopen(device, "w");
  if (!fp) {
    int err = errno;

    std::cout << "failed to open device for write :" << device << std::endl;

    return err;
  }

  rc = fputs(value, fp);
  fclose(fp);

  if (rc < 0) {

    std::cout << "failed to write device" << device << std::endl;

    return ENOENT;
  } else {
    return 0;
  }
}

// Get BIC Configuration
static int bic_get_config(uint8_t host, bic_config_t *cfg) {

  int ret;
  std::vector<uint8_t> cmdData{0x15, 0xA0, 0x0, 0x03,
                               0x0,  0x0,  0x0, 0x0}; // IANA ID
  std::vector<uint8_t> respData;
  uint8_t rlen = 0;

  sendIPMBRequest(host, NETFN_OEM_1S_REQ, CMD_OEM_1S_GET_CONFIG, cmdData,
                  respData);
  if (DEBUG) {
   std::cout << "bic_get_config  : host " << (int)host << std::endl;

  std::cerr << "bic_get_config Response:\n" << std::flush;
  for (int i = 0; i < respData.size(); i++) {
    printf("0x%x :", respData[i]);
  }

  printf("\n");

  }
   uint8_t *temp = respData.data();

  *(uint8_t *)cfg = temp[3];

  std::cout.flush();

  return ret;
}

// Set BIC Configuration
static int bic_set_config(uint8_t host, bic_config_t *cfg) {
  int ret;
  std::vector<uint8_t> cmdData{0x15, 0xA0, 0x0, 0x04,
                               0x0,  0x0,  0x0, 0x0}; // IANA ID
  std::vector<uint8_t> respData;
  uint8_t rlen = 0;

   if (DEBUG)
    std::cout << "bic_set_config for host:" << (int)host << std::endl;

  sendIPMBRequest(host, NETFN_OEM_1S_REQ, CMD_OEM_1S_SET_CONFIG, cmdData,
                  respData);

   std::cerr << "bic_get_config Response:\n" << std::flush;
  for (int i = 0; i < respData.size(); i++) {
    printf("0x%x :", respData[i]);
  }
  std::cout.flush();

  return ret;
}

// Enable POST buffer for the server in given slot
static int post_enable(uint8_t host) {
  int ret;
  bic_config_t config = {0};


  ret = bic_get_config(host, &config);
  if (ret) {

    std::cout << "post_enable: bic_get_config failed for host :" << host
              << std::endl;

    return ret;
  }

  if (0 == config.post) \
{

    config.post = 1;
    ret = bic_set_config(host, &config);
    if (ret) {

      std::cout << "post_enable: bic_set_config failed in host:" << host
                << std::endl;

      return ret;
    }
  }
}


// Display the given POST code using GPIO port
static int post_display(uint8_t status) {
  char path[64] = {0};
  int ret;
  char *val;

  //std::cout << "post_display: status is" << std::hex < status << std::endl;

  if (DEBUG)
  printf("Postcode : 0%x", status);

  sprintf(path, GPIO_VAL, GPIO_POSTCODE_0);

  if (BIT(status, 0)) {
    val = "1";
  } else {
    val = "0";
  }

  ret = write_device(path, val);
  if (ret) {
    goto post_exit;
  }

  sprintf(path, GPIO_VAL, GPIO_POSTCODE_1);
  if (BIT(status, 1)) {
    val = "1";
  } else {
    val = "0";
  }

  ret = write_device(path, val);
  if (ret) {
    goto post_exit;
  }

  sprintf(path, GPIO_VAL, GPIO_POSTCODE_2);
  if (BIT(status, 2)) {
    val = "1";
  } else {
    val = "0";
  }

  ret = write_device(path, val);
  if (ret) {
    goto post_exit;
  }

  sprintf(path, GPIO_VAL, GPIO_POSTCODE_3);
  if (BIT(status, 3)) {
    val = "1";
  } else {
    val = "0";
  }

  ret = write_device(path, val);
  if (ret) {
    goto post_exit;
  }

  sprintf(path, GPIO_VAL, GPIO_POSTCODE_4);
  if (BIT(status, 4)) {
    val = "1";
  } else {
    val = "0";
  }

  ret = write_device(path, val);
  if (ret) {
    goto post_exit;
  }

  sprintf(path, GPIO_VAL, GPIO_POSTCODE_5);
  if (BIT(status, 5)) {
    val = "1";
  } else {
    val = "0";
  }

  ret = write_device(path, val);
  if (ret) {
    goto post_exit;
  }

  sprintf(path, GPIO_VAL, GPIO_POSTCODE_6);
  if (BIT(status, 6)) {
    val = "1";
  } else {
    val = "0";
  }

  ret = write_device(path, val);
  if (ret) {
    goto post_exit;
  }

  sprintf(path, GPIO_VAL, GPIO_POSTCODE_7);
  if (BIT(status, 7)) {
    val = "1";
  } else {
    val = "0";
  }

  ret = write_device(path, val);
  if (ret) {
    goto post_exit;
  }

post_exit:
  if (ret) {
    std::cout << "write_device failed for" << path << std::endl;
    return -1;
  } else {
    return 0;
  }
}


// Helper Functions
static int read_device(const char *device, int *value) {
  FILE *fp;
  int rc;

  if (DEBUG)
  std::cout << "read_device()" << std::endl;

  fp = fopen(device, "r");
  if (!fp) {
    int err = errno;
    std::cout << "failed to open device" << device << std::endl;
    return err;
  }

  rc = fscanf(fp, "%d", value);
  fclose(fp);
  if (rc != 1) {
    std::cout << "failed to read device" << device << std::endl;
    return ENOENT;
  } else {
    return 0;
  }
}

static uint8_t is_bic_ready(uint8_t host) {
  int val;
  char path[64] = {0};

  std::cout << "is_bic_ready in host" << host << std::endl;

  if (host < 1 || host > 4) {
    return 0;
  }

  sprintf(path, GPIO_VAL, gpio_bic_ready[host]);
  if (read_device(path, &val)) {
    return 0;
  }

  if (val == 0x0) {
    return 1;
  } else {
    return 0;
  }
}
int is_debug_card_prsnt(uint8_t *status) {
  int val;
  char path[64] = {0};

   if (DEBUG)
    std::cout << "is_debug_card_prsnt()" << std::endl;

  sprintf(path, GPIO_VAL, GPIO_DBG_CARD_PRSNT);

  if (read_device(path, &val)) {
    return -1;
  }

  if (val == 0x0) {
    *status = 1;
  } else {
    *status = 0;
  }

  return 0;
}

static void readPostcode(uint8_t postcode) {
int ret;
uint8_t host = 1;  // Set Host0 always TODO : Need to apply Kumar IPMI patch.

std::cout << "postcode_new:" << (int)postcode << "host:" <<(int)host << std::endl;

 switch (host) {
  case 1:
    fb_ipmi::miscPface->set_property("Value", val);
    fb_ipmi::miscP1face->set_property("Value", val);
    break;
  case 2:
    fb_ipmi::miscP2face->set_property("Value", val);
    break;
  }
  ret = post_display(postcode);
  if (ret) {
   std::cout << "GPIO write error" << std::endl;
  }
}


static void BICInit() {
  int MAX_HOST = 2;
  for (int host = 0; host < MAX_HOST; host++) {
    std::cerr<<" Get Gpio Config for the host " <<  host << "\n"<<std::flush;
    getGpioConfiguration(host);
    setGpioConfiguration(host);
    std::cerr<<"Aftere set Gpio Config\n"<<std::flush;
    getGpioConfiguration(host);
    // Enable postcode in Bridge IC through IPMI Interface
    post_enable(host);
  }
}
}; // namespace fb_ipmi

int main(int argc, char *argv[]) {
  std::cerr << "Facebook Misc Ipmi service ...\n";

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

  fb_ipmi::conn->request_name("xyz.openbmc_project.State.Boot.Raw");
  fb_ipmi::conn->request_name("xyz.openbmc_project.State.Host0.Boot.Raw");
  fb_ipmi::conn->request_name("xyz.openbmc_project.State.Host1.Boot.Raw");


  // Host1 postcode Interfac
  fb_ipmi::miscP1face =
      postServer.add_interface("/xyz/openbmc_project/state/host0/boot/raw",
                               "xyz.openbmc_project.State.Host0.Boot.Raw");

  fb_ipmi::miscP1face->register_property(
      "Value", int(0), sdbusplus::asio::PropertyPermission::readWrite);

  fb_ipmi::miscP1face->initialize();

  // Host2 postcode Interface
  fb_ipmi::miscP2face =
      postServer.add_interface("/xyz/openbmc_project/state/host1/boot/raw",
                               "xyz.openbmc_project.State.Host1.Boot.Raw");

  fb_ipmi::miscP2face->register_property(
      "Value", int(0), sdbusplus::asio::PropertyPermission::readWrite);

  fb_ipmi::miscP2face->initialize();
  fb_ipmi::io.run();

  return 0;
}
