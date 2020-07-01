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
#include <gpiod.hpp>
#include <nlohmann/json.hpp>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string_view>
#include <vector>

namespace fb_ipmi
{
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

static bool requestGPIOEvents(
    const std::string& name, const std::function<void()>& handler,
    gpiod::line& gpioLine,
    boost::asio::posix::stream_descriptor& gpioEventDescriptor)
{
    // Find the GPIO line
    gpioLine = gpiod::find_line(name);
    if (!gpioLine)
    {
        std::cerr << "Failed to find the " << name << " line\n";
        return false;
    }

    try
    {
        gpioLine.request(
            {"fb-yv2-misc", gpiod::line_request::EVENT_BOTH_EDGES});
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
            handler();
        });
    return true;
}

static void updateHandSwitchPosition()
{
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

static void HandSwitch1Handler()
{
    gpiod::line_event gpioLineEvent = HandSwitch1Line.event_read();

    if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE)
    {
        updateHandSwitchPosition();
    }
    if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE)
    {
        updateHandSwitchPosition();
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
    gpiod::line_event gpioLineEvent = HandSwitch2Line.event_read();

    if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE)
    {
        updateHandSwitchPosition();
    }
    if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE)
    {
        updateHandSwitchPosition();
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
    gpiod::line_event gpioLineEvent = HandSwitch3Line.event_read();

    if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE)
    {
        updateHandSwitchPosition();
    }
    if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE)
    {
        updateHandSwitchPosition();
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
    gpiod::line_event gpioLineEvent = HandSwitch4Line.event_read();

    if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE)
    {
        updateHandSwitchPosition();
    }
    else if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE)
    {
        updateHandSwitchPosition();
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

static void powerButtonHandler()
{
    gpiod::line_event gpioLineEvent = powerButtonLine.event_read();

    if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE)
    {
        std::cerr << "power button pressed = 1 \n";
        updateHandSwitchPosition();
        miscIface->set_property("PowerButtonPressed", 1);
    }
    else if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE)
    {
        std::cerr << "power button pressed = 0 \n";
        miscIface->set_property("PowerButtonPressed", 0);
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
    gpiod::line_event gpioLineEvent = resetButtonLine.event_read();

    if (gpioLineEvent.event_type == gpiod::line_event::FALLING_EDGE)
    {
        std::cerr << "Reset button pressed = 1 \n";
        updateHandSwitchPosition();
        miscIface->set_property("ResetButtonPressed", 1);
    }
    else if (gpioLineEvent.event_type == gpiod::line_event::RISING_EDGE)
    {
        std::cerr << "Reset button pressed = 0 \n";
        miscIface->set_property("ResetButtonPressed", 0);
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

int sendIPMBRequest(uint8_t host, uint8_t netFn, uint8_t cmd,
                    std::vector<uint8_t>& cmdData,
                    std::vector<uint8_t>& respData)
{

    auto method =
        conn->new_method_call("xyz.openbmc_project.Ipmi.Channel.Ipmb",
                              "/xyz/openbmc_project/Ipmi/Channel/Ipmb",
                              "org.openbmc.Ipmb", "sendRequest");
    method.append(host, netFn, lun, cmd, cmdData);

    auto reply = conn->call(method);
    if (reply.is_method_error())
    {
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

static void setGpioConfiguration(uint8_t host)
{
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

static void BICInit()
{
    int MAX_HOST = 2;
    for (int host = 0; host < MAX_HOST; host++)
    {
        setGpioConfiguration(host);
    }
}
}; // namespace fb_ipmi

#define GPIO_VAL "/sys/class/gpio/gpio%d/value"
#define GPIO_BASE_NUM 280
#define GPIO_DBG_CARD_PRSNT (139 + GPIO_BASE_NUM)

#define CMD_OEM_1S_GET_CONFIG 0x0E
#define CMD_OEM_1S_SET_CONFIG 0x10
#define NETFN_OEM_1S_REQ 0x38
#define CMD_OEM_1S_GET_POST_BUF 0x12

#define GPIO_POSTCODE_0 (48 + GPIO_BASE_NUM)
#define GPIO_POSTCODE_1 (49 + GPIO_BASE_NUM)
#define GPIO_POSTCODE_2 (50 + GPIO_BASE_NUM)
#define GPIO_POSTCODE_3 (51 + GPIO_BASE_NUM)
#define GPIO_POSTCODE_4 (124 + GPIO_BASE_NUM)
#define GPIO_POSTCODE_5 (125 + GPIO_BASE_NUM)
#define GPIO_POSTCODE_6 (126 + GPIO_BASE_NUM)
#define GPIO_POSTCODE_7 (127 + GPIO_BASE_NUM)

#define UART1_TXD (1 << 22)
#define UART2_TXD (1 << 30)
#define UART3_TXD (1 << 22)
#define UART4_TXD (1 << 30)

#define GPIO_UART_SEL0 32 + GPIO_BASE_NUM
#define GPIO_UART_SEL1 33 + GPIO_BASE_NUM
#define GPIO_UART_SEL2 34 + GPIO_BASE_NUM
#define GPIO_UART_RX 35 + GPIO_BASE_NUM

#define MAX_IPMB_RES_LEN 0x10 // TODO
#define MAX_VALUE_LEN 50      // TODO

#define BIT(value, index) ((value >> index) & 1)

#define AST_GPIO_BASE 0x1e780000
#define GPIO_AB_AA_Z_Y 0x1e0
#define PAGE_SIZE 0x1000
#define PIN_CTRL1_OFFSET 0x80
#define PIN_CTRL2_OFFSET 0x84
#define AST_SCU_BASE 0x1e6e2000

#define GPIO_I2C_SLOT1_ALERT_N 106 + GPIO_BASE_NUM
#define GPIO_I2C_SLOT2_ALERT_N 107 + GPIO_BASE_NUM
#define GPIO_I2C_SLOT3_ALERT_N 108 + GPIO_BASE_NUM
#define GPIO_I2C_SLOT4_ALERT_N 109 + GPIO_BASE_NUM

static constexpr bool DEBUG = true;
static constexpr uint8_t lun = 0;
static void* m_gpio_hand_sw = NULL;
static uint8_t m_pos = 0xff;

using respType =
    std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>;

std::shared_ptr<sdbusplus::asio::connection> conn;

const static uint32_t gpio_bic_ready[] = {
    0, GPIO_I2C_SLOT1_ALERT_N, GPIO_I2C_SLOT2_ALERT_N, GPIO_I2C_SLOT3_ALERT_N,
    GPIO_I2C_SLOT4_ALERT_N};

typedef struct _bic_config_t
{
    union
    {
        struct
        {
            uint8_t sol : 1;
            uint8_t post : 1;
            uint8_t rsvd : 6;
        };
        uint8_t config;
    };
} bic_config_t;

enum
{
    HAND_SW_SERVER1 = 1,
    HAND_SW_SERVER2,
    HAND_SW_SERVER3,
    HAND_SW_SERVER4,
    HAND_SW_BMC
};

enum
{
    IPMB_BUS_SLOT1 = 1,
    IPMB_BUS_SLOT2 = 3,
    IPMB_BUS_SLOT3 = 5,
    IPMB_BUS_SLOT4 = 7,
};

#define MAX_NUM_FRUS 6
enum
{
    FRU_ALL = 0,
    FRU_SLOT1 = 1,
    FRU_SLOT2 = 2,
    FRU_SLOT3 = 3,
    FRU_SLOT4 = 4,
    FRU_SPB = 5,
    FRU_NIC = 6,
    FRU_BMC = 7,
};

// Read the Front Panel Hand Switch and return the position
int pal_get_hand_sw_physically(uint8_t* pos)
{
    int gpio_fd;
    void* gpio_reg;
    uint8_t loc;

    std::cout << "pal_get_hand_sw_physically()" << std::endl;

    if (!m_gpio_hand_sw)
    {
        gpio_fd = open("/dev/mem", O_RDWR | O_SYNC);
        if (gpio_fd < 0)
        {
            std::cout << "failed to open /dev/mem" << std::endl;
            return -1;
        }

        gpio_reg = mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
                        gpio_fd, AST_GPIO_BASE);
        m_gpio_hand_sw = (char*)gpio_reg + GPIO_AB_AA_Z_Y;
        close(gpio_fd);
    }

    loc = (((*(volatile uint32_t*)m_gpio_hand_sw) >> 20) & 0xF) %
          5; // GPIOAA[7:4]

    switch (loc)
    {
        case 0:
            *pos = HAND_SW_SERVER1;
            break;
        case 1:
            *pos = HAND_SW_SERVER2;
            break;
        case 2:
            *pos = HAND_SW_SERVER3;
            break;
        case 3:
            *pos = HAND_SW_SERVER4;
            break;
        default:
            *pos = HAND_SW_BMC;
            break;
    }

    std::cout << "Exit : pal_get_hand_sw_physically()" << std::endl;

    return 0;
}

int pal_get_hand_sw(uint8_t* pos)
{
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

// Helper Functions
static int read_device(const char* device, int* value)
{
    FILE* fp;
    int rc;

    std::cout << "read_device()" << std::endl;

    fp = fopen(device, "r");
    if (!fp)
    {
        int err = errno;
        std::cout << "failed to open device" << device << std::endl;
        return err;
    }

    rc = fscanf(fp, "%d", value);
    fclose(fp);
    if (rc != 1)
    {
        std::cout << "failed to read device" << device << std::endl;
        return ENOENT;
    }
    else
    {
        return 0;
    }
}

static int write_device(const char* device, const char* value)
{
    FILE* fp;
    int rc;

    std::cout << "write_device()" << std::endl;

    fp = fopen(device, "w");
    if (!fp)
    {
        int err = errno;

        std::cout << "failed to open device for write :" << device << std::endl;

        return err;
    }

    rc = fputs(value, fp);
    fclose(fp);

    if (rc < 0)
    {

        std::cout << "failed to write device" << device << std::endl;

        return ENOENT;
    }
    else
    {
        return 0;
    }
}

int is_debug_card_prsnt(uint8_t* status)
{
    int val;
    char path[64] = {0};

    std::cout << "is_debug_card_prsnt()" << std::endl;

    sprintf(path, GPIO_VAL, GPIO_DBG_CARD_PRSNT);

    if (read_device(path, &val))
    {
        return -1;
    }

    if (val == 0x0)
    {
        *status = 1;
    }
    else
    {
        *status = 0;
    }

    return 0;
}

static int get_ipmb_bus_id(uint8_t slot_id)
{
    int bus_id;

    std::cout << "get_ipmb_bus_id()" << std::endl;

    switch (slot_id)
    {
        case FRU_SLOT1:
            bus_id = IPMB_BUS_SLOT1;
            break;
        case FRU_SLOT2:
            bus_id = IPMB_BUS_SLOT2;
            break;
        case FRU_SLOT3:
            bus_id = IPMB_BUS_SLOT3;
            break;
        case FRU_SLOT4:
            bus_id = IPMB_BUS_SLOT4;
            break;
        default:
            bus_id = -1;
            break;
    }

    return bus_id;
}

uint8_t is_bic_ready(uint8_t slot_id)
{
    int val;
    char path[64] = {0};

    std::cout << "is_bic_ready()" << std::endl;

    if (slot_id < 1 || slot_id > 4)
    {
        return 0;
    }

    sprintf(path, GPIO_VAL, gpio_bic_ready[slot_id]);
    if (read_device(path, &val))
    {
        return 0;
    }

    if (val == 0x0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int bic_ipmb_wrapper(uint8_t slot_id, uint8_t netFn, uint8_t cmd,
                     std::vector<uint8_t>& cmdData, uint8_t cmdLen,
                     std::vector<uint8_t>& respData, uint8_t* respLen)
{

    uint8_t ret;
    uint8_t bicAddr;
    int retry = 0;

    std::cout << "bic_ipmb_wrapper()" << std::endl;

    if (!is_bic_ready(slot_id))
    {
        return -1;
    }

    ret = get_ipmb_bus_id(slot_id);
    if (ret < 0)
    {
        std::cout << "bic_ipmb_wrapper: Wrong Slot ID" << slot_id << std::endl;
        return ret;
    }

    bicAddr = (uint8_t)ret;

    cmdData.reserve(cmdData.size() + cmdLen);

    std::copy(&cmdData[0], &cmdData[cmdLen], back_inserter(cmdData));

    auto method =
        conn->new_method_call("xyz.openbmc_project.Ipmi.Channel.Ipmb",
                              "/xyz/openbmc_project/Ipmi/Channel/Ipmb",
                              "org.openbmc.Ipmb", "sendRequest");
    method.append(bicAddr, netFn, lun, cmd, cmdData);

    if (DEBUG)
    {
        std::cout << "BIC NetFn:cmd " << (int)netFn << ":" << (int)cmd << "\n";
        std::cout << "BIC req data: ";
        for (auto d : cmdData)
        {
            std::cout << d << " ";
        }
        std::cout << "\n";
    }

    while (retry < 3)
    {

        auto reply = conn->call(method);
        if (reply.is_method_error())
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "Error reading from BIC");
            return -1;
        }

        respType resp;
        reply.read(resp);

        respData = std::move(
            std::get<std::remove_reference_t<decltype(respData)>>(resp));

        if (respData.empty())
        {
            if (!is_bic_ready(slot_id))
            {
                std::cout << "BIC is busy" << std::endl;
                break;
            }
            retry++;
            usleep(20000);
        }
        else
        {
            break;
        }
    }

    if (respData.empty())
    {
        std::cout << "bic_ipmb_wrapper: Zero bytes received, retry " << retry
                  << std::endl;
        return -1;
    }
    if (DEBUG)
    {
        std::cout << "BIC resp data: " << std::flush;
        for (auto d : respData)
        {
            std::cout << int(d) << " " << std::flush;
        }
        std::cout << "\n" << std::flush;
    }

    return 0;
}

// Display the given POST code using GPIO port
static int post_display(uint8_t status)
{
    char path[64] = {0};
    int ret;
    char* val;

    std::cout << "post_display: status is" << status << std::endl;

    sprintf(path, GPIO_VAL, GPIO_POSTCODE_0);

    if (BIT(status, 0))
    {
        val = "1";
    }
    else
    {
        val = "0";
    }

    ret = write_device(path, val);
    if (ret)
    {
        goto post_exit;
    }

    sprintf(path, GPIO_VAL, GPIO_POSTCODE_1);
    if (BIT(status, 1))
    {
        val = "1";
    }
    else
    {
        val = "0";
    }

    ret = write_device(path, val);
    if (ret)
    {
        goto post_exit;
    }

    sprintf(path, GPIO_VAL, GPIO_POSTCODE_2);
    if (BIT(status, 2))
    {
        val = "1";
    }
    else
    {
        val = "0";
    }

    ret = write_device(path, val);
    if (ret)
    {
        goto post_exit;
    }

    sprintf(path, GPIO_VAL, GPIO_POSTCODE_3);
    if (BIT(status, 3))
    {
        val = "1";
    }
    else
    {
        val = "0";
    }

    ret = write_device(path, val);
    if (ret)
    {
        goto post_exit;
    }

    sprintf(path, GPIO_VAL, GPIO_POSTCODE_4);
    if (BIT(status, 4))
    {
        val = "1";
    }
    else
    {
        val = "0";
    }

    ret = write_device(path, val);
    if (ret)
    {
        goto post_exit;
    }

    sprintf(path, GPIO_VAL, GPIO_POSTCODE_5);
    if (BIT(status, 5))
    {
        val = "1";
    }
    else
    {
        val = "0";
    }

    ret = write_device(path, val);
    if (ret)
    {
        goto post_exit;
    }

    sprintf(path, GPIO_VAL, GPIO_POSTCODE_6);
    if (BIT(status, 6))
    {
        val = "1";
    }
    else
    {
        val = "0";
    }

    ret = write_device(path, val);
    if (ret)
    {
        goto post_exit;
    }

    sprintf(path, GPIO_VAL, GPIO_POSTCODE_7);
    if (BIT(status, 7))
    {
        val = "1";
    }
    else
    {
        val = "0";
    }

    ret = write_device(path, val);
    if (ret)
    {
        goto post_exit;
    }

post_exit:
    if (ret)
    {
        std::cout << "write_device failed for" << path << std::endl;
        return -1;
    }
    else
    {
        return 0;
    }
}

// Handle the received post code, for now display it on debug card
int post_handle(uint8_t slot, uint8_t status)
{
    uint8_t prsnt, pos;
    int ret;

    std::cout << "post_handle()" << std::endl;

    // Check for debug card presence
    ret = is_debug_card_prsnt(&prsnt);
    if (ret)
    {
        return ret;
    }

    // No debug card  present, return
    if (!prsnt)
    {
        // return 0;
    }

    // Get the hand switch position
    ret = pal_get_hand_sw(&pos);
    if (ret)
    {
        return ret;
    }

    // If the give server is not selected, return
    if (pos != slot)
    {
        return 0;
    }

    // Display the post code in the debug card
    ret = post_display(status);
    if (ret)
    {
        return ret;
    }

    return 0;
}

// Read POST Buffer
int bic_get_post_buf(uint8_t slot_id, uint8_t* buf, uint8_t* len)
{
    std::vector<uint8_t> tbuf = {0x15, 0xA0, 0x00}; // IANA ID
    std::vector<uint8_t> rbuf;
    uint8_t rlen = 0;
    int ret;

    std::cout << "bic_get_post_buf()" << std::endl;
#if 0
    ret = bic_ipmb_wrapper(slot_id, NETFN_OEM_1S_REQ, CMD_OEM_1S_GET_POST_BUF,
                           tbuf, 0x03, rbuf, &rlen);

    // Ignore IANA ID
    memcpy(buf, &rbuf[3], rlen - 3);

    *len = rlen - 3;
#endif
    return ret;
}

// Get the last post code of the given slot
int post_get_last(uint8_t slot, uint8_t* status)
{

    int ret;
    uint8_t buf[MAX_IPMB_RES_LEN] = {0x0};
    uint8_t len;

    std::cout << "post_get_last()" << std::endl;

    ret = bic_get_post_buf(slot, buf, &len);
    if (ret)
    {
        // return ret;
    }

    // The post buffer is LIFO and the first byte gives the latest post code
    //*status = buf[0];

    return 0;
}

// Get BIC Configuration
int bic_get_config(uint8_t slot_id, bic_config_t* cfg)
{
    std::vector<uint8_t> tbuf = {0x15, 0xA0, 0x00}; // IANA ID
    std::vector<uint8_t> rbuf;
    uint8_t rlen = 0;
    int ret;

    std::cout << "bic_get_config()" << std::endl;
#if 0
    ret = bic_ipmb_wrapper(slot_id, NETFN_OEM_1S_REQ, CMD_OEM_1S_GET_CONFIG,
                           tbuf, 0x03, rbuf, &rlen);

    // Ignore IANA ID
    *(uint8_t*)cfg = rbuf[3];

#endif

    return ret;
}

// Set BIC Configuration
int bic_set_config(uint8_t slot_id, bic_config_t* cfg)
{
    std::vector<uint8_t> tbuf = {0x15, 0xA0, 0x00}; // IANA ID
    std::vector<uint8_t> rbuf;
    uint8_t rlen = 0;
    int ret;

    std::cout << "bic_set_config()" << std::endl;
#if 0

    tbuf[3] = *(uint8_t*)cfg;



    ret = bic_ipmb_wrapper(slot_id, NETFN_OEM_1S_REQ, CMD_OEM_1S_SET_CONFIG,
                           tbuf, 0x04, rbuf, &rlen);
#endif
    return ret;
}

// Enable POST buffer for the server in given slot
int post_enable(uint8_t slot)
{
    int ret;
    bic_config_t config = {0};

    std::cout << "post_enable()" << std::endl;

    ret = bic_get_config(slot, &config);
    if (ret)
    {

        std::cout << "post_enable: bic_get_config failed for fru:" << slot
                  << std::endl;

        // return ret;
    }

    // if (0 == config.post)
    {
        config.post = 1;
        ret = bic_set_config(slot, &config);
        if (ret)
        {

            std::cout << "post_enable: bic_set_config failed" << std::endl;

            return ret;
        }
    }
}

// Debug Card's UART and BMC/SoL port share UART port and need to enable only
// one TXD i.e. either BMC's TXD or Debug Port's TXD.
static int control_sol_txd(uint8_t slot, uint8_t dis_tx)
{
    uint32_t scu_fd;
    uint32_t ctrl;
    void* scu_reg;
    void* scu_pin_ctrl1;
    void* scu_pin_ctrl2;

    scu_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (scu_fd < 0)
    {
        std::cout << "control_sol_txd" << std::endl;
        return -1;
    }

    std::cout << "control_sol_txd()" << std::endl;

    scu_reg = mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, scu_fd,
                   AST_SCU_BASE);
    scu_pin_ctrl1 = (char*)scu_reg + PIN_CTRL1_OFFSET;
    scu_pin_ctrl2 = (char*)scu_reg + PIN_CTRL2_OFFSET;

    switch (slot)
    {
        case 1:
            // Disable UART1's TXD and enable others
            ctrl = *(volatile uint32_t*)scu_pin_ctrl2;
            ctrl = (dis_tx) ? ctrl & ~UART1_TXD : ctrl | UART1_TXD;
            ctrl |= UART2_TXD;
            *(volatile uint32_t*)scu_pin_ctrl2 = ctrl;

            ctrl = *(volatile uint32_t*)scu_pin_ctrl1;
            ctrl |= UART3_TXD | UART4_TXD;
            *(volatile uint32_t*)scu_pin_ctrl1 = ctrl;
            break;
        case 2:
            // Disable UART2's TXD and enable others
            ctrl = *(volatile uint32_t*)scu_pin_ctrl2;
            ctrl |= UART1_TXD;
            ctrl = (dis_tx) ? ctrl & ~UART2_TXD : ctrl | UART2_TXD;
            *(volatile uint32_t*)scu_pin_ctrl2 = ctrl;

            ctrl = *(volatile uint32_t*)scu_pin_ctrl1;
            ctrl |= UART3_TXD | UART4_TXD;
            *(volatile uint32_t*)scu_pin_ctrl1 = ctrl;
            break;
        case 3:
            // Disable UART3's TXD and enable others
            ctrl = *(volatile uint32_t*)scu_pin_ctrl2;
            ctrl |= UART1_TXD | UART2_TXD;
            *(volatile uint32_t*)scu_pin_ctrl2 = ctrl;

            ctrl = *(volatile uint32_t*)scu_pin_ctrl1;
            ctrl = (dis_tx) ? ctrl & ~UART3_TXD : ctrl | UART3_TXD;
            ctrl |= UART4_TXD;
            *(volatile uint32_t*)scu_pin_ctrl1 = ctrl;
            break;
        case 4:
            // Disable UART4's TXD and enable others
            ctrl = *(volatile uint32_t*)scu_pin_ctrl2;
            ctrl |= UART1_TXD | UART2_TXD;
            *(volatile uint32_t*)scu_pin_ctrl2 = ctrl;

            ctrl = *(volatile uint32_t*)scu_pin_ctrl1;
            ctrl |= UART3_TXD;
            ctrl = (dis_tx) ? ctrl & ~UART4_TXD : ctrl | UART4_TXD;
            *(volatile uint32_t*)scu_pin_ctrl1 = ctrl;
            break;
        default:
            // Any other slots we need to enable all TXDs
            ctrl = *(volatile uint32_t*)scu_pin_ctrl2;
            ctrl |= UART1_TXD | UART2_TXD;
            *(volatile uint32_t*)scu_pin_ctrl2 = ctrl;

            ctrl = *(volatile uint32_t*)scu_pin_ctrl1;
            ctrl |= UART3_TXD | UART4_TXD;
            *(volatile uint32_t*)scu_pin_ctrl1 = ctrl;
            break;
    }

    munmap(scu_reg, PAGE_SIZE);
    close(scu_fd);

    std::cout << "Exit :: control_sol_txd()" << std::endl;

    return 0;
}

// Switch the UART mux to the given slot
int pal_switch_uart_mux(uint8_t slot)
{
    char* gpio_uart_sel0;
    char* gpio_uart_sel1;
    char* gpio_uart_sel2;
    char* gpio_uart_rx;
    char path[64] = {0};
    uint8_t prsnt;
    int ret;

    std::cout << "pal_switch_uart_mux()" << std::endl;

    ret = is_debug_card_prsnt(&prsnt);
    if (ret)
    {
        goto uart_exit;
    }

    // Refer the UART select table in schematic
    switch (slot)
    {
        case HAND_SW_SERVER1:
            gpio_uart_sel2 = "0";
            gpio_uart_sel1 = "0";
            gpio_uart_sel0 = "0";
            // gpio_uart_rx = (prsnt) ? "0" : "1";
            break;
        case HAND_SW_SERVER2:
            gpio_uart_sel2 = "0";
            gpio_uart_sel1 = "0";
            gpio_uart_sel0 = "1";
            // gpio_uart_rx = (prsnt) ? "0" : "1";
            break;
        case HAND_SW_SERVER3:
            gpio_uart_sel2 = "0";
            gpio_uart_sel1 = "1";
            gpio_uart_sel0 = "0";
            // gpio_uart_rx = (prsnt) ? "0" : "1";
            break;
        case HAND_SW_SERVER4:
            gpio_uart_sel2 = "0";
            gpio_uart_sel1 = "1";
            gpio_uart_sel0 = "1";
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
    if (ret)
    {
        // goto uart_exit;
    }

    // Enable Debug card path
    sprintf(path, GPIO_VAL, GPIO_UART_SEL2);
    ret = write_device(path, gpio_uart_sel2);
    if (ret)
    {
        // goto uart_exit;
    }

    sprintf(path, GPIO_VAL, GPIO_UART_SEL1);
    ret = write_device(path, gpio_uart_sel1);
    if (ret)
    {
        // goto uart_exit;
    }

    sprintf(path, GPIO_VAL, GPIO_UART_SEL0);
    ret = write_device(path, gpio_uart_sel0);
    if (ret)
    {
        // goto uart_exit;
    }

    sprintf(path, GPIO_VAL, GPIO_UART_RX);
    ret = write_device(path, gpio_uart_rx);
    if (ret)
    {
        // goto uart_exit;
    }

uart_exit:
    if (ret)
    {
        std::cout << "pal_switch_uart_mux: write_device failed:" << path
                  << std::endl;
        return ret;
    }
    else
    {
        return 0;
    }
}

void* debug_card_handler(void* threadid)
{
    long tid;
    int curr = -1;
    int prev = -1;
    int ret;
    uint8_t prsnt = 0;
    uint8_t pos, usb_pos;
    uint8_t prev_pos = 0xff, prev_phy_pos = 0xff;
    uint8_t lpc;
    uint8_t status;
    char str[8];
    tid = (long)threadid;
    std::cout << "debug_card_handler! Thread ID, " << tid << std::endl;

    while (1)

    {

        ret = pal_get_hand_sw_physically(&pos);
        if (ret)
        {
            goto debug_card_out;
        }

        if (pos == prev_phy_pos)
        {
            goto get_hand_sw_cache;
        }

        usleep(10000);
        ret = pal_get_hand_sw_physically(&pos);
        if (ret)
        {
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
        if (ret)
        {
            goto debug_card_out;
        }

        if (pos == prev_pos)
        {
            // goto debug_card_prs; //TODO
        }
        m_pos = pos;

#if 0
    ret = pal_switch_usb_mux(pos);
    if (ret) {
      goto debug_card_out;
    }
#endif

        ret = pal_switch_uart_mux(pos);
        if (ret)
        {
            goto debug_card_out;
        }

#if 0
debug_card_prs:
    ret = pal_get_usb_sw(&usb_pos);
    if (ret) {
      goto debug_card_out;
    }

    if (usb_pos <= MAX_NUM_SLOTS) {
      if (!pal_is_slot_server(usb_pos) || (!pal_get_server_power(usb_pos, &status) && (status != SERVER_POWER_ON))) {
        pal_enable_usb_mux(USB_MUX_OFF);
      } else {
        pal_enable_usb_mux(USB_MUX_ON);
      }
    }

#endif
        // Check if debug card present or not
        ret = is_debug_card_prsnt(&prsnt);
        if (ret)
        {
            goto debug_card_out;
        }
        curr = prsnt;

        // Check if Debug Card was either inserted or removed
        if (curr != prev)
        {
            if (!curr)
            {
                // Debug Card was removed
                std::cout << "Debug Card Extraction" << std::endl;
            }
            else
            {
                // Debug Card was inserted
                std::cout << "Debug Card Insertion" << std::endl;
            }
        }

        if ((pos == prev_pos) && (curr == prev))
        {
            goto debug_card_out;
        }

        // Enable POST codes for all slots
        ret = post_enable(pos);
        if (ret)
        {
            // goto debug_card_done;
        }

        // Get last post code and display it
        ret = post_get_last(pos, &lpc);
        if (ret)
        {
            // goto debug_card_done;
        }

        ret = post_handle(pos, lpc);
        if (ret)
        {
            // goto debug_card_out;
        }

    debug_card_done:
        prev = curr;
        prev_pos = pos;
    debug_card_out:
        if (curr == 1)
        {
            usleep(500000); // TODO
        }
        else
            sleep(1);

        sleep(800);
    }
    pthread_exit(NULL);
}

int main(int argc, char* argv[])
{
    int rc;
    pthread_t handlePostcode;
    rc = pthread_create(&handlePostcode, NULL, debug_card_handler, NULL);

    if (rc)
    {
        std::cout << "Error:unable to create Postcode thread," << rc
                  << std::endl;
        exit(-1);
    }

    std::cerr << "Facebook Misc Ipmi service ....\n";

    fb_ipmi::conn = std::make_shared<sdbusplus::asio::connection>(fb_ipmi::io);

    fb_ipmi::conn->request_name("xyz.openbmc_project.Misc.Ipmi");

#if 0

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

#endif
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
