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

int hostPos = 10;

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

  int temp = position;
  if ((temp == 1) || (temp == 6)) {
    hostPos = position = 1;
  } else if ((temp == 2) || (temp == 7)) {
    hostPos = position = 2;
  } else if ((temp == 3) || (temp == 8)) {
    hostPos = position = 3;
  } else if ((temp == 4) || (temp == 9)) {
    hostPos = position = 4;
  } else if ((temp == 4) || (temp == 9)) {
    hostPos = position = 5;
  }

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
    miscIface->set_property("PowerButton_Host1", false);
  } else if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE) {
    std::cerr << "power button pressed = 0 \n";
    miscIface->set_property("PowerButton_Host1", true);
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
    miscIface->set_property("ResetButton_Host1", false);
  } else if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE) {
    std::cerr << "Reset button pressed = 0 \n";
    miscIface->set_property("ResetButton_Host1", true);
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

static bool getPowerGoodStatus(uint8_t host) {
  int netFn = 0x38;
  int cmd = 3;
  std::vector<uint8_t> cmdData{0x15, 0xa0, 0};
  std::vector<uint8_t> respData;

  sendIPMBRequest(host, netFn, cmd, cmdData, respData);
  uint8_t GpiosStatus = respData[3];
  bool pwrGdStatusFromIPMI =
      (GpiosStatus & CPUPwrGdMask) && (GpiosStatus & PCHPwrGdMask);

  return pwrGdStatusFromIPMI;
}

static void powerGoodHandler() {
  std::cerr << "Check power good handler\n";
  boost::asio::steady_timer timer{fb_ipmi::io, std::chrono::milliseconds{1000}};
  timer.wait();
  miscIface->set_property("Power_Good_Host1", getPowerGoodStatus(0));
  miscIface->set_property("Power_Good_Host2", getPowerGoodStatus(1));
  powerGoodHandler();
}

inline static sdbusplus::bus::match::match powerOkEventMonitor() {
  auto pulseEventMatcherCallback = [](sdbusplus::message::message &msg) {
    std::string thresholdInterface;
    boost::container::flat_map<std::string, std::variant<std::string>>
        propertiesChanged;
    msg.read(thresholdInterface, propertiesChanged);

    if (propertiesChanged.empty()) {
      return;
    }
    std::string event = propertiesChanged.begin()->first;
    std::string value =
        std::get<std::string>(propertiesChanged.begin()->second);

    if ((event != "CurrentPowerState")) {
      return;
    }
    while (value ==
           "xyz.openbmc_project.State.Chassis.PowerState.PowerGoodWait") {
      miscIface->set_property("Power_Good_Host1", getPowerGoodStatus(0));
      miscIface->set_property("Power_Good_Host2", getPowerGoodStatus(1));
    }
  };

  sdbusplus::bus::match::match pulseEventMatcher(
      static_cast<sdbusplus::bus::bus &>(*conn),
      "type='signal',interface='org.freedesktop.DBus.Properties',member='"
      "PropertiesChanged',arg0namespace='xyz.openbmc_project.State.Chassis'",
      std::move(pulseEventMatcherCallback));

  return pulseEventMatcher;
}

#define GPIO_VAL "/sys/class/gpio/gpio%d/value"
#define GPIO_BASE_NUM 792

#define GPIO_DBG_CARD_PRSNT (139 + GPIO_BASE_NUM)

#define UART1_TXD (1 << 22)
#define UART2_TXD (1 << 30)
#define UART3_TXD (1 << 22)
#define UART4_TXD (1 << 30)

#define GPIO_UART_SEL0 32 + GPIO_BASE_NUM
#define GPIO_UART_SEL1 33 + GPIO_BASE_NUM
#define GPIO_UART_SEL2 34 + GPIO_BASE_NUM
#define GPIO_UART_RX 35 + GPIO_BASE_NUM

#define AST_GPIO_BASE 0x1e780000
#define GPIO_AB_AA_Z_Y 0x1e0
#define PAGE_SIZE 0x1000
#define PIN_CTRL1_OFFSET 0x80
#define PIN_CTRL2_OFFSET 0x84
#define AST_SCU_BASE 0x1e6e2000

#define MAX_VALUE_LEN 50 // TODO

static std::shared_ptr<sdbusplus::asio::dbus_interface> miscPface;
static std::shared_ptr<sdbusplus::asio::dbus_interface> miscP1face;
static std::shared_ptr<sdbusplus::asio::dbus_interface> miscP2face;

static constexpr bool DEBUG = false;

static void *m_gpio_hand_sw = NULL;
static uint8_t m_pos = 0xff;

enum {
  HAND_SW_SERVER1 = 1,
  HAND_SW_SERVER2,
  HAND_SW_SERVER3,
  HAND_SW_SERVER4,
  HAND_SW_BMC
};

static int write_device(const char *device, const char *value) {
  FILE *fp;
  int rc;

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

// Helper Functions
static int read_device(const char *device, int *value) {
  FILE *fp;
  int rc;

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

// Debug Card's UART and BMC/SoL port share UART port and need to enable only
// one TXD i.e. either BMC's TXD or Debug Port's TXD.
static int control_sol_txd(uint8_t slot, uint8_t dis_tx) {
  uint32_t scu_fd;
  uint32_t ctrl;
  void *scu_reg;
  void *scu_pin_ctrl1;
  void *scu_pin_ctrl2;

  scu_fd = open("/dev/mem", O_RDWR | O_SYNC);
  if (scu_fd < 0) {
    std::cout << "control_sol_txd" << std::endl;
    return -1;
  }

  std::cout << "control_sol_txd()" << std::endl;

  scu_reg = mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, scu_fd,
                 AST_SCU_BASE);
  scu_pin_ctrl1 = (char *)scu_reg + PIN_CTRL1_OFFSET;
  scu_pin_ctrl2 = (char *)scu_reg + PIN_CTRL2_OFFSET;

  switch (slot) {
  case 1:
    // Disable UART1's TXD and enable others
    ctrl = *(volatile uint32_t *)scu_pin_ctrl2;
    ctrl = (dis_tx) ? ctrl & ~UART1_TXD : ctrl | UART1_TXD;
    ctrl |= UART2_TXD;
    *(volatile uint32_t *)scu_pin_ctrl2 = ctrl;

    ctrl = *(volatile uint32_t *)scu_pin_ctrl1;
    ctrl |= UART3_TXD | UART4_TXD;
    *(volatile uint32_t *)scu_pin_ctrl1 = ctrl;
    break;
  case 2:
    // Disable UART2's TXD and enable others
    ctrl = *(volatile uint32_t *)scu_pin_ctrl2;
    ctrl |= UART1_TXD;
    ctrl = (dis_tx) ? ctrl & ~UART2_TXD : ctrl | UART2_TXD;
    *(volatile uint32_t *)scu_pin_ctrl2 = ctrl;

    ctrl = *(volatile uint32_t *)scu_pin_ctrl1;
    ctrl |= UART3_TXD | UART4_TXD;
    *(volatile uint32_t *)scu_pin_ctrl1 = ctrl;
    break;
  case 3:
    // Disable UART3's TXD and enable others
    ctrl = *(volatile uint32_t *)scu_pin_ctrl2;
    ctrl |= UART1_TXD | UART2_TXD;
    *(volatile uint32_t *)scu_pin_ctrl2 = ctrl;

    ctrl = *(volatile uint32_t *)scu_pin_ctrl1;
    ctrl = (dis_tx) ? ctrl & ~UART3_TXD : ctrl | UART3_TXD;
    ctrl |= UART4_TXD;
    *(volatile uint32_t *)scu_pin_ctrl1 = ctrl;
    break;
  case 4:
    // Disable UART4's TXD and enable others
    ctrl = *(volatile uint32_t *)scu_pin_ctrl2;
    ctrl |= UART1_TXD | UART2_TXD;
    *(volatile uint32_t *)scu_pin_ctrl2 = ctrl;

    ctrl = *(volatile uint32_t *)scu_pin_ctrl1;
    ctrl |= UART3_TXD;
    ctrl = (dis_tx) ? ctrl & ~UART4_TXD : ctrl | UART4_TXD;
    *(volatile uint32_t *)scu_pin_ctrl1 = ctrl;
    break;
  default:
    // Any other slots we need to enable all TXDs
    ctrl = *(volatile uint32_t *)scu_pin_ctrl2;
    ctrl |= UART1_TXD | UART2_TXD;
    *(volatile uint32_t *)scu_pin_ctrl2 = ctrl;

    ctrl = *(volatile uint32_t *)scu_pin_ctrl1;
    ctrl |= UART3_TXD | UART4_TXD;
    *(volatile uint32_t *)scu_pin_ctrl1 = ctrl;
    break;
  }

  munmap(scu_reg, PAGE_SIZE);
  close(scu_fd);

  std::cout << "Exit :: control_sol_txd()" << std::endl;

  return 0;
}

int is_debug_card_prsnt(uint8_t *status) {
  int val;
  char path[64] = {0};

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

// Read the Front Panel Hand Switch and return the position
static int pal_get_hand_sw_physically(uint8_t *pos) {
  int gpio_fd;
  void *gpio_reg;
  uint8_t loc;

  *pos = hostPos;

  std::cout << "Exit : pal_get_hand_sw_physically()" << std::endl;

  return 0;
}

static int pal_get_hand_sw(uint8_t *pos) {
  char value[MAX_VALUE_LEN] = {0};
  uint8_t loc;
  int ret;

  std::cout << "pal_get_hand_sw()" << std::endl;

#if 0
    ret = kv_get("spb_hand_sw", value, NULL, 0); //TODO
    if (!ret)
    {
        loc = atoi(value);
        if ((loc > HAND_SW_BMC) || (loc < HAND_SW_SERVER1))
        {
            return pal_get_hand_sw_physically(pos);
        }

        pal_get_hand_sw_physically(pos);
        *pos = loc;
        return 0;
    }
#endif

  return pal_get_hand_sw_physically(pos);
}

// Switch the UART mux to the given slot
static int pal_switch_uart_mux(uint8_t slot) {
  const char *gpio_uart_sel0;
  const char *gpio_uart_sel1;
  const char *gpio_uart_sel2;
  const char *gpio_uart_rx;
  char path[64] = {0};
  uint8_t prsnt;
  int ret;

  std::cout << "pal_switch_uart_mux()" << std::endl;

  ret = is_debug_card_prsnt(&prsnt);
  if (ret) {
    goto uart_exit;
  }

  // Refer the UART select table in schematic
  switch (slot) {
  case HAND_SW_SERVER1:
    gpio_uart_sel2 = "0";
    gpio_uart_sel1 = "0";
    gpio_uart_sel0 = "0";
    // gpio_uart_rx = (prsnt) ? "0" : "1";
    if (prsnt)
      gpio_uart_rx = "0";
    else
      gpio_uart_rx = "1";
    break;
  case HAND_SW_SERVER2:
    gpio_uart_sel2 = "0";
    gpio_uart_sel1 = "0";
    gpio_uart_sel0 = "1";
    if (prsnt)
      gpio_uart_rx = "0";
    else
      gpio_uart_rx = "1";
    // gpio_uart_rx = (prsnt) ? "0" : "1";
    break;
  case HAND_SW_SERVER3:
    gpio_uart_sel2 = "0";
    gpio_uart_sel1 = "1";
    gpio_uart_sel0 = "0";
    if (prsnt)
      gpio_uart_rx = "0";
    else
      gpio_uart_rx = "1";
    // gpio_uart_rx = (prsnt) ? "0" : "1";
    break;
  case HAND_SW_SERVER4:
    gpio_uart_sel2 = "0";
    gpio_uart_sel1 = "1";
    gpio_uart_sel0 = "1";
    if (prsnt)
      gpio_uart_rx = "0";
    else
      gpio_uart_rx = "1";
    // gpio_uart_rx = (prsnt) ? "0" : "1";
    break;
  default:
    // for all other cases, assume BMC
    gpio_uart_sel2 = "1";
    gpio_uart_sel1 = "0";
    gpio_uart_sel0 = "0";
    gpio_uart_rx = "1";
    break;
  }

  //  Diable TXD path from BMC to avoid conflict with SoL
  ret = control_sol_txd(slot, prsnt);
  if (ret) {
    goto uart_exit;
  }

  // Enable Debug card path
  sprintf(path, GPIO_VAL, GPIO_UART_SEL2);
  ret = write_device(path, gpio_uart_sel2);
  if (ret) {
    goto uart_exit;
  }

  sprintf(path, GPIO_VAL, GPIO_UART_SEL1);
  ret = write_device(path, gpio_uart_sel1);
  if (ret) {
    goto uart_exit;
  }

  sprintf(path, GPIO_VAL, GPIO_UART_SEL0);
  ret = write_device(path, gpio_uart_sel0);
  if (ret) {
    goto uart_exit;
  }

  sprintf(path, GPIO_VAL, GPIO_UART_RX);
  ret = write_device(path, gpio_uart_rx);
  if (ret) {
    goto uart_exit;
  }

uart_exit:
  if (ret) {
    std::cout << "pal_switch_uart_mux: write_device failed:" << path
              << std::endl;
    return ret;
  } else {
    return 0;
  }
}

int curr = -1;
int prev = -1;
int ret;
uint8_t prsnt = 0;
uint8_t pos, usb_pos;
uint8_t prev_pos = 0xff, prev_phy_pos = 0xff;
uint8_t lpc;
uint8_t status;
char str[8];

void *debug_card_handler(void *threadid) {

  long tid;

  tid = (long)threadid;
  std::cout << "debug_card_handler " << std::endl;

  while (1) {

    ret = pal_get_hand_sw_physically(&pos);
    if (ret) {
      goto debug_card_out;
    }

    if (pos == prev_phy_pos) {
      goto get_hand_sw_cache;
    }

    usleep(10000);
    ret = pal_get_hand_sw_physically(&pos);
    if (ret) {
      goto debug_card_out;
    }

    prev_phy_pos = pos;
#if 0
        sprintf(str, "%u", pos);
        ret = kv_set("spb_hand_sw", str, 0, 0);
        if (ret)
        {
            goto debug_card_out;
        }
#endif
  get_hand_sw_cache:
    ret = pal_get_hand_sw(&pos);
    if (ret) {
      goto debug_card_out;
    }

    if (pos == prev_pos) {
      goto debug_card_prs; // TODO
    }
    m_pos = pos;

#if 0
    ret = pal_switch_usb_mux(pos);
    if (ret) {
      goto debug_card_out;
    }
#endif

    ret = pal_switch_uart_mux(pos);
    if (ret) {
      goto debug_card_out;
    }

  debug_card_prs:

    // Check if debug card present or not
    ret = is_debug_card_prsnt(&prsnt);
    if (ret) {
      goto debug_card_out;
    }
    curr = prsnt;

    // Check if Debug Card was either inserted or removed
    if (curr != prev) {
      if (!curr) {
        // Debug Card was removed
        std::cout << "Debug Card Extraction" << std::endl;
      } else {
        // Debug Card was inserted
        std::cout << "Debug Card Insertion" << std::endl;
      }
    }

    if ((pos == prev_pos) && (curr == prev)) {
      goto debug_card_out;
    }

  debug_card_done:
    prev = curr;
    prev_pos = pos;
  debug_card_out:
    if (curr == 1) {
      usleep(500000); // TODO
    } else
      sleep(1);

    sleep(800);
  }

  pthread_exit(NULL);
}

} // namespace fb_ipmi

int main(int argc, char *argv[]) {
  std::cerr << "Facebook Misc Ipmi service ....\n";

  fb_ipmi::conn = std::make_shared<sdbusplus::asio::connection>(fb_ipmi::io);

  fb_ipmi::conn->request_name("xyz.openbmc_project.Chassis.Event");

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

  std::cerr << "After function powerGoodHandler \n";
  // Power Control Service
  sdbusplus::asio::object_server miscServer =
      sdbusplus::asio::object_server(fb_ipmi::conn);

#if 1
  // Power Control Interface
  fb_ipmi::miscIface =
      miscServer.add_interface("/xyz/openbmc_project/Chassis/Event",
                               "xyz.openbmc_project.Chassis.Event");

#endif

  fb_ipmi::miscIface->register_property(
      "PowerButton_Host1", bool(true),
      sdbusplus::asio::PropertyPermission::readWrite);
  fb_ipmi::miscIface->register_property(
      "ResetButton_Host1", bool(true),
      sdbusplus::asio::PropertyPermission::readWrite);
  fb_ipmi::miscIface->register_property(
      "Power_Good_Host1", bool(false),
      sdbusplus::asio::PropertyPermission::readWrite);
  fb_ipmi::miscIface->register_property(
      "Power_Good_Host2", bool(false),
      sdbusplus::asio::PropertyPermission::readWrite);
  fb_ipmi::miscIface->register_property(
      "Position", int(0), sdbusplus::asio::PropertyPermission::readWrite);
  fb_ipmi::miscIface->initialize();

  sdbusplus::bus::match::match pulseEventMonitor =
      fb_ipmi::powerOkEventMonitor();

  fb_ipmi::io.run();

  return 0;
}
