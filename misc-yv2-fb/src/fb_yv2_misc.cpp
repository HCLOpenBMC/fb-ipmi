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
#include <string_view>
#include <fstream>
#include <iostream>
#include <vector>
#include <iterator>

//using namespace std;
std::shared_ptr<sdbusplus::asio::connection> conn;
static boost::asio::io_service io;
static constexpr uint8_t lun = 0;

using respType =
	std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>;
	//std::tuple<int, std::vector<uint8_t>>;
	//std::tuple<int, uint8_t, uint8_t, uint8_t>;
int sendIPMBRequest(uint8_t host, uint8_t netFn, uint8_t cmd,
                    std::vector<uint8_t> &cmdData,
                    std::vector<uint8_t> &respData) {

  std::cerr << "SendIPMBRequest data: " << +host << " " << +netFn << " " << +lun << " " << +cmd << " 0x" ;
  std::copy( cmdData.begin(), cmdData.end(), std::ostream_iterator<int>( std::cerr << std::hex, " 0x" ) );
  std::cerr << "\n";  

  auto method = conn->new_method_call("xyz.openbmc_project.Ipmi.Channel.Ipmb",
                                      "/xyz/openbmc_project/Ipmi/Channel/Ipmb",
                                      "org.openbmc.Ipmb", "sendRequest");
  method.append(host, netFn, lun, cmd, cmdData);

	//std::cerr << "Method print: " << method << "\n";

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
   
  int x = respData[0];
  int y = respData[1];
  std::cerr << "Respdata in sendIPMBReq = " << x << " " << y << "\n"; 
  return 0;
}

/*string uint8_vector_to_hex_string(vector<uint8_t>& v) {
    stringstream ss;
    ss << std::hex << std::setfill('0');
    vector<uint8_t>::const_iterator it;

    for (it = v.begin(); it != v.end(); it++) {
        ss << "\\x" << std::setw(2) << static_cast<unsigned>(*it);
    }

    return ss.str();
}
*/

int send_data(uint8_t slot_id, uint8_t net_fun, int cmd_id, std::vector<uint8_t> &sendData, uint32_t offset)
{
		
    std::vector<uint8_t> cmdData{0x15, 0xa0, 0};
    std::vector<uint8_t> respData;
	uint16_t total_len = 0;
    int retries = 1;
	int bios_update=0;
    //int retries = 3;
    int ret = 0;

	// Fill the component for which firmware is requested
    //cmdData.push_back(slot_id);
    cmdData.push_back(bios_update);
    cmdData.push_back((offset) & 0xFF);
    cmdData.push_back((offset >> 8) & 0xFF);
    cmdData.push_back((offset >> 16) & 0xFF);
    cmdData.push_back((offset >> 24) & 0xFF);
    cmdData.push_back((sendData.size()) & 0xFF);
    cmdData.push_back((sendData.size() >> 8) & 0xFF);
    cmdData.insert(cmdData.end(), std::move_iterator(sendData.begin()),
      							   std::move_iterator(sendData.end()));
	//total_len = cmdData.size();
    //cmdData.insert(cmdData.begin(), total_len);
	
	//std::cerr << "send_data values : 0x";
    //std::copy( cmdData.begin(), cmdData.end(), std::ostream_iterator<int>( std::cerr << std::hex, " 0x" ) );
    std::cerr << "send_data size " << cmdData.size() << "\n";
	//const auto hex_str = uint8_vector_to_hex_string(cmdData);
	//std::cerr << "send_data values= " << hex_str << "\n";

    while (retries != 0)
    {
        sendIPMBRequest(slot_id, net_fun, cmd_id, cmdData, respData);
		std::cerr << "respDataSize = " << respData.size() << "\n";
		//std::cerr << "respData = " << respData[0];// << " " << respData[1] << " " << respData[2] << " " << respData[3] << "\n "; 
#if 0
        uint8_t retStatus = respData[0];
		
		if (retStatus == 0)
		{
	    	std::cerr<< "Data Sent successfully \n"; 
	    	break;
		}
		else if (retStatus == 0x80)
		{
			sleep(1);
            std::cerr<< "Write Flash Error!! slot:" << slot_id << " Offset:" << offset << " len:" << sendData.size() << "Retrying.....\n"; 
		}
		else if (retStatus == 0x81)
		{
			sleep(1);
            std::cerr<< "Power status check Fail!! slot:" << slot_id << " Offset:" << offset << " len:" << sendData.size() << "Retrying.....\n"; 
		}
		else if (retStatus == 0x82)
		{
			sleep(1);
            std::cerr<< "Data length Error!! slot:" << slot_id << " Offset:" << offset << " len:" << sendData.size() << "Retrying.....\n"; 
		}
		else if (retStatus == 0x83)
		{
			sleep(1);
            std::cerr<< "Flash Erase Error!! slot:" << slot_id << " Offset:" << offset << " len:" << sendData.size() << "Retrying.....\n"; 
		}
		else 
		{
			sleep(1);
			std::cerr << "Invalid Data... Retrying....\n";
		}
		
#endif
		retries --;
    }
	
	if (retries == 0)
	{
		std::cerr << "Error!!! Not able to send bios data!!! \n"; 
		return -1;
	}

    return 0;
}

int get_chksum_fw(uint8_t slot_id, uint8_t net_fun, uint32_t cmd_get_fw_chksum, uint32_t offset, std::vector<uint8_t> &respData)
{
    std::vector<uint8_t> cmdData{0x15, 0xa0, 0};
    int retries = 3;
    int count =12;

    // Fill the component for which firmware is requested
    cmdData.push_back(slot_id);
    cmdData.push_back((offset) & 0xFF);
    cmdData.push_back((offset >> 8) & 0xFF);
    cmdData.push_back((offset >> 16) & 0xFF);
    cmdData.push_back((offset >> 24) & 0xFF);
    cmdData.push_back(count & 0xFF);
    cmdData.push_back((count >> 8) & 0xFF);
    cmdData.push_back((count >> 16) & 0xFF);
    cmdData.push_back((count >> 24) & 0xFF);

    while (retries !=0)
    {
        sendIPMBRequest(slot_id, net_fun, cmd_get_fw_chksum, cmdData, respData);
        if (respData.size() !=6)		
		{
	    	sleep(1);
	    	std::cerr<< "Checksum not obtained properly for slot:" << slot_id << " Offset:" << offset << " len:" << count << "Retrying.....\n"; 
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

int bios_update(uint8_t slot_id, const char* image_path)
{

    std::cerr << "Bios update bin file path " << image_path <<"\n";

    // Read the binary data from bin file
    int count	= 0;
    int cmd_id 	= 9;
    uint8_t net_fun = 0x38;
    uint32_t offset = 0;
    uint32_t ipmb_write_max = 224;

    // Open the file
    std::streampos fileSize;
    std::ifstream file(image_path, std::ios::in|std::ios::binary|std::ios::ate);
    
    if (file.is_open())
    {   
    
        // Get its size
        fileSize = file.tellg();
        std::cerr << "Filesize " << fileSize <<"\n";
        file.seekg(0, std::ios::beg);

        //while (offset <= fileSize) 
        while (offset <= 1000) 
        {
			// Read the data
            std::vector<uint8_t> fileData(ipmb_write_max);
            file.read((char*) &fileData[0], ipmb_write_max);

            std::cerr << "Offset " << offset <<"\n";
			//std::cerr << "Bios Image value= ";
            //std::copy( fileData.begin(), fileData.end(), std::ostream_iterator<int>( std::cerr << std::hex, " " ) );
            //std::cerr << "Size " << fileData.size() << "\n";
			//const auto hex_str = uint8_vector_to_hex_string(fileData);
			//std::cerr << "Bios image values= " << hex_str << "\n";


        	int ret = send_data(slot_id, net_fun, cmd_id, fileData, offset);
			if (ret != 0)
			{
				return -1;
			}
            offset += ipmb_write_max;
        }


        // Check for bios image
        uint32_t offset_d = 0;
        file.seekg(0, std::ios::beg);
        int bios_verify_pkt_size = (32*1024);
        std::cerr << "Verify Bios image...\n";

        while (offset_d <= fileSize)
        {
            // Read the data
            std::vector<int> chksum(bios_verify_pkt_size);
            file.read((char*) &chksum[0], bios_verify_pkt_size);
            std::cerr << "Offset_d " << offset_d << "\n";

            uint32_t tcksum = 0;
            for (int i = 0; i < chksum.size(); i++)
            {
                tcksum += chksum[i];
            }
            std::cerr << "tcksum = " << tcksum << "\n";
	    
			std::vector<std::uint8_t> calChksum((std::uint8_t*)&tcksum
                                   , (std::uint8_t*)&(tcksum) + sizeof(std::uint32_t));

	    	// Get the checksum value from firmware
	    	uint8_t retValue;
	    	uint32_t cmd_get_fw_chksum = 0xA;
	   		std::vector<uint8_t> fwChksumData;
	    	
			retValue = get_chksum_fw(slot_id, net_fun, cmd_get_fw_chksum, offset_d, fwChksumData);
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
      				std::cerr <<"checksum does not match, offset:" << offset_d << " Calculated chksum:" << calChksum[i] << " FW Chksum:" << fwChksumData[i] <<"\n";
      				return -1;
	    		}
			}    	    
            offset_d += bios_verify_pkt_size;
        }


        file.close();

    }
    else std::cerr << "Unable to open file";

    return 0;
}

int main(int argc, char* argv[])
{
	conn = std::make_shared<sdbusplus::asio::connection>(io);

    // Get the bin file
    const char* bin_file = argv[1];
    //const char * slot_id = argv[1];
    uint8_t slot_id = 1; //(int) (argv[2]);
	
    std::cerr << "Bin File Path " << bin_file << "\n";
    std::cerr << "IPMB Based Bios Upgrade Started for slot#" << +slot_id << "\n";
    int ret = bios_update(slot_id, bin_file);
    if (ret != 0) 
    {
	std::cerr << "IPMB Based Bios upgrade failed for slot#" << +slot_id << "\n";
	return -1;
    }

    std::cerr << "IPMB Based Bios upgrade completed successfully for slot#" << +slot_id << "\n";
	io.run();
    return 0;
}

