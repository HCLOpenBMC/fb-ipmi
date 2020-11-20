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
#include "fb_yv2_misc.hpp"

#include <filesystem>
#include <string_view>
#include <fstream>
#include <iostream>
#include <vector>
#include <iterator>


std::shared_ptr<sdbusplus::asio::connection> conn;
static boost::asio::io_service io;
static constexpr uint8_t lun = 0;

using respType =
    std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>;

/*
Function Name    : sendIPMBRequest
Description      : Send data to target through Ipmb
*/
int sendIPMBRequest(uint8_t host, uint8_t netFn, uint8_t cmd,
                    std::vector<uint8_t> &cmdData,
                    std::vector<uint8_t> &respData)
{

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

    respData.insert(respData.begin(), std::get<4>(resp));

    if (respData.size() <= 0)
    {
        return -1;
    }
  return 0;
}


/*
Function Name    : sendFWData
Description      : Form vectors with required data to send
*/
int sendFWData(uint8_t slotId, uint8_t netFun, int cmdFw,
               std::vector<uint8_t> &sendData, uint32_t offset, uint8_t updateCmd)
{

    std::vector<uint8_t> cmdData{IANA_ID_0, IANA_ID_1, IANA_ID_2};
    std::vector<uint8_t> respData;
    int retries = MAX_RETRY;
    int ret = 0;

    // Fill the component for which firmware is requested
    cmdData.push_back(updateCmd);
    cmdData.push_back((offset) & 0xFF); //--> google and find out if there is any way to swap or endian change directly without >>/&
    cmdData.push_back((offset >> 8) & 0xFF);
    cmdData.push_back((offset >> 16) & 0xFF);
    cmdData.push_back((offset >> 24) & 0xFF);
    cmdData.push_back((sendData.size()) & 0xFF);
    cmdData.push_back((sendData.size() >> 8) & 0xFF);
    cmdData.insert(cmdData.end(), std::move_iterator(sendData.begin()),
                                     std::move_iterator(sendData.end()));

    while (retries != 0)
    {
        sendIPMBRequest(slotId, netFun, cmdFw, cmdData, respData);
        uint8_t retStatus = respData[0];

        if (retStatus == 0)
        {
            std::cerr<< "Data Sent successfully \n";
            break;
        }
        else if (retStatus == 0x80)
        {
            sleep(0.001);
            std::cerr<< "Write Flash Error!! slot:" << slotId << " Offset:" << offset << " len:" << sendData.size() << "Retrying.....\n";
        }
        else if (retStatus == 0x81)
        {
            sleep(0.001);
            std::cerr<< "Power status check Fail!! slot:" << slotId << " Offset:" << offset << " len:" << sendData.size() << "Retrying.....\n";
        }
        else if (retStatus == 0x82)
        {
            sleep(0.001);
            std::cerr<< "Data length Error!! slot:" << slotId << " Offset:" << offset << " len:" << sendData.size() << "Retrying.....\n";
        }
        else if (retStatus == 0x83)
        {
            sleep(0.001);
            std::cerr<< "Flash Erase Error!! slot:" << slotId << " Offset:" << offset << " len:" << sendData.size() << "Retrying.....\n";
        }
        else
        {
            sleep(0.001);
            std::cerr << "Invalid Data... Retrying....\n";
        }

        retries --;
    }

    if (retries == 0)
    {
        std::cerr << "Error!!! Not able to send bios data!!! \n";
        return -1;
    }

    return 0;
}

/*
Function Name    : getChksumFW
Description      : Get the checksum value of bios image
*/
int getChksumFW(uint8_t slotId, uint8_t netFun, uint32_t cmdGetFWChksum,
                uint32_t offset, uint32_t len, std::vector<uint8_t> &respData,
                uint8_t updateCmd)
{
    std::vector<uint8_t> cmdData{IANA_ID_0, IANA_ID_1, IANA_ID_2};
    int retries = MAX_RETRY;

    // Fill the component for which firmware is requested
    cmdData.push_back(updateCmd);
    cmdData.push_back((offset) & 0xFF);
    cmdData.push_back((offset >> 8) & 0xFF);
    cmdData.push_back((offset >> 16) & 0xFF);
    cmdData.push_back((offset >> 24) & 0xFF);
    cmdData.push_back(len & 0xFF);
    cmdData.push_back((len >> 8) & 0xFF);
    cmdData.push_back((len >> 16) & 0xFF);
    cmdData.push_back((len >> 24) & 0xFF);

    while (retries !=0)
    {
        sendIPMBRequest(slotId, netFun, cmdGetFWChksum, cmdData, respData);
        if (respData.size() !=6)
        {
            sleep(0.001);
            std::cerr<< "Checksum not obtained properly for slot:" << slotId << " Offset:" << offset << " len:" << len << "Retrying.....\n";
            retries --;
        }
    }

    if (retries == 0)
    {
        std::cerr << "Failed to get the Checksum value from firmware.. \n";
        return -1;
    }

    return 0;
}

/*
Function Name    : meRecovery
Description      : Set Me to recovery mode
*/
int meRecovery(uint8_t slotId, uint8_t netFun, uint8_t cmdME, uint8_t mode)
{
    std::vector<uint8_t> cmdData{IANA_ID_0, IANA_ID_1, IANA_ID_2, BIC_INTF_ME,
                                 ME_RECOVERY_CMD_0, ME_RECOVERY_CMD_1,
                                 ME_RECOVERY_CMD_2, ME_RECOVERY_CMD_3,
                                 ME_RECOVERY_CMD_4, mode};
    std::vector<uint8_t> respData;
    int retries = MAX_RETRY;

    while (retries != 0)
    {
        sendIPMBRequest(slotId, netFun, cmdME, cmdData, respData);
        if (respData.size() !=6)
        {
            sleep(0.001);
            std::cerr << "ME is not set into recovery mode.. Retrying... \n";
        }
        else if (respData[3] != cmdData[3])
        {
            sleep(0.001);
            std::cerr << "Interface not valid.. Retrying...  \n";
        }
        else if (respData[0] == 0)
        {
            sleep(0.001);
            std::cerr << "ME recovery mode -> Completion Status set.. \n";
            break;
        }
        else if (respData[0] != 0)
        {
            sleep(0.001);
            std::cerr << "ME recovery mode -> Completion Status not set.. Retrying..\n";
        }
        else
        {
            sleep(0.001);
            std::cerr << "Invalid data or command... \n";
        }
        retries --;
    }

    if (retries == 0)
    {
        std::cerr << "Failed to set ME to recovery mode.. \n";
        return -1;
    }

    // Verify whether ME went to recovery mode
    std::vector<uint8_t> meData{IANA_ID_0, IANA_ID_1, IANA_ID_2, BIC_INTF_ME,
                                VERIFY_ME_RECV_CMD_0, VERIFY_ME_RECV_CMD_1};
    std::vector<uint8_t> meResp;
    retries = MAX_RETRY;

    while (retries != 0)
    {
        sendIPMBRequest(slotId, netFun, cmdME, meData, meResp);
        if (meResp[3] != meData[3])
        {
            sleep(0.001);
            std::cerr << "Interface not valid.. Retrying...  \n";
        }
        else if ((mode == 0x1) && (meResp[1] == 0x81) && (meResp[2] == 0x02))
        {
            return 0;
        }
        retries --;
    }

    if (retries == 0)
    {
        std::cerr << "Failed to set ME to recovery mode in self tests.. \n";
        return -1;
    }


    return 0;
}

/*
Function Name   : updateFw
Description     : Send data to respective target for FW udpate
Param: slotId   : Slot Id
Param: imagePath: Binary image path
Param: updateCmd: cmd Id to find the target (BIOS, CPLD, VR, ME)
*/
int updateFw(uint8_t slotId, const char* imagePath, uint8_t updateCmd)
{

    std::cerr << "Bios update bin file path " << imagePath <<"\n";

    // Read the binary data from bin file
    int count             = 0x0;
    int cmdFw             = 0x9;
    uint8_t cmdME         = 0x2;
    uint32_t offset       = 0x0;
    uint8_t netFun        = 0x38;
    uint8_t recoveryMode  = 0x1;
    uint32_t ipmbWriteMax = 128;


    // Set ME to recovery mode
    int ret_val = meRecovery(slotId, netFun, cmdME, recoveryMode);
    if (ret_val != 0)
    {
        std::cerr << "Me set to recovery mode failed\n";
        return -1;
    }

    // Open the file
    std::streampos fileSize;
    std::ifstream file(imagePath, std::ios::in|std::ios::binary|std::ios::ate);

    if (file.is_open())
    {

        // Get its size
        fileSize = file.tellg();
        std::cerr << "Total Filesize " << fileSize <<"\n";
        file.seekg(0, std::ios::beg);
        int i = 1;

        while (offset < fileSize)
        {

            // count details
            uint32_t count = ipmbWriteMax;

            if ((offset+ipmbWriteMax) >= (i * BIOS_64k_SIZE))
            {
                count = (i * BIOS_64k_SIZE) - offset;
                i++;
            }

            // Read the data
            std::vector<uint8_t> fileData(ipmbWriteMax);
            file.read((char*) &fileData[0], ipmbWriteMax);

            // Send data
            int ret = sendFWData(slotId, netFun, cmdFw, fileData, offset, updateCmd);
            if (ret != 0)
            {
                std::cerr << "Firmware update Failed at offset " << offset << "\n" ;
                return -1;
            }

            // Update counter
            offset += count;
        }


        // Check for bios image
        uint32_t offset_d = 0;
        uint32_t biosVerifyPktSize = BIOS_32k_SIZE;

        file.seekg(0, std::ios::beg);
        std::cerr << "Verify Bios image...\n";

        while (offset_d < fileSize)
        {
            // Read the data
            std::vector<int> chksum(biosVerifyPktSize);
            file.read((char*) &chksum[0], biosVerifyPktSize);

            // Calculate checksum
            uint32_t tcksum = 0;
            for (int i = 0; i < chksum.size(); i++)
            {
                tcksum += chksum[i];
            }

            std::vector<std::uint8_t> calChksum((std::uint8_t*)&tcksum,
                     (std::uint8_t*)&(tcksum) + sizeof(std::uint32_t));


            // Get the checksum value from firmware
            uint8_t retValue;
            uint32_t cmdGetFWChksum = 0xA;
               std::vector<uint8_t> fwChksumData;

            retValue = getChksumFW(slotId, netFun, cmdGetFWChksum,
                                   offset_d, biosVerifyPktSize,
                                   fwChksumData, updateCmd);
            if (retValue != 0)
            {
                std::cerr << "Failed to get the Checksum value!! \n";
                return -1;
            }


            for (uint8_t i=0; i <= calChksum.size(); i++)
            {
                // Compare both and see if they match or not
                if (fwChksumData[i] != calChksum[i])
                {
                      std::cerr <<"checksum does not match, offset:" << offset_d << " Calculated chksum:" << +calChksum[i] << " FW Chksum:" << +fwChksumData[i] <<"\n";
                      return -1;
                }
            }
            offset_d += biosVerifyPktSize;
        }


        file.close();

    }
    else std::cerr << "Unable to open file";

    return 0;
}

int main(int argc, char* argv[])
{
    conn = std::make_shared<sdbusplus::asio::connection>(io);

    // Get the arguments
    const char* binFile = argv[1];
    uint8_t slotId      = (int) (argv[2]);
    uint8_t updateCmd   = (int) (argv[3]);

    std::cerr << "Bin File Path " << binFile << "\n";
    std::cerr << "IPMB Based Bios Upgrade Started for slot#" << +slotId << "\n";
    int ret = updateFw(slotId, binFile, updateCmd);
    if (ret != 0)
    {
        std::cerr << "IPMB Based Bios upgrade failed for slot#" << +slotId << "\n";
        return -1;
    }

    std::cerr << "IPMB Based Bios upgrade completed successfully for slot#" << +slotId << "\n";

    io.run();
    return 0;
}

