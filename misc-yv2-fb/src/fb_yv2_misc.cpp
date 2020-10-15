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

#include <boost/asio/io_service.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <gpiod.hpp>
#include <nlohmann/json.hpp>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/asio/object_server.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string_view>
#include <vector>

static bool getPowerGoodStatus(uint8_t host)
{
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

static void powerGoodHandler()
{
    std::cerr << "Check power good handler\n";
    boost::asio::steady_timer timer{fb_ipmi::io,
                                    std::chrono::milliseconds{1000}};
    timer.wait();
    miscIface->set_property("Power_Good_Host1", getPowerGoodStatus(0));
    miscIface->set_property("Power_Good_Host2", getPowerGoodStatus(1));
    powerGoodHandler();
}

int main(int argc, char* argv[])
{

    std::cerr << "IPMB Based Bios Upgrade Started....\n";

    return 0;
}
