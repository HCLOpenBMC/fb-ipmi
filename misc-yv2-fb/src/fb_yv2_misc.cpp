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

int sendIPMBRequest(uint8_t host, uint8_t netFn, uint8_t cmd,
                    std::vector<uint8_t> &cmdData,
                    std::vector<uint8_t> &respData) {

  /*std::cerr << "SendIPMBRequest data: " << +host << " " << +netFn << " " << +lun << " " << +cmd << " 0x" ;
  std::copy( cmdData.begin(), cmdData.end(), std::ostream_iterator<int>( std::cerr << std::hex, " 0x" ) );
  std::cerr << "\n";  
*/
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

  //std::cerr << "Resp data " << std::get<0>(resp) << "\n ";
  std::cerr << "Resp data 1 " << std::get<0>(resp) << " 2 " << std::get<1>(resp) << " 3 " << std::get<2>(resp) << " 4 " << std::get<3>(resp) << " 5 " << std::get<4>(resp) << " vector  \n ";

  respData =
      std::move(std::get<std::remove_reference_t<decltype(respData)>>(resp));
   
  //std::copy( respData.begin(), respData.end(), std::ostream_iterator<uint8_t>( std::cerr  << std::hex, " " ) );

  respData.insert(respData.begin(), std::get<4>(resp));
  std::copy( respData.begin(), respData.end(), std::ostream_iterator<uint8_t>( std::cerr  << std::hex, " " ) );
 
  return 0;
}


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
    /*cmdData.push_back(bios_update);
    cmdData.push_back((offset) & 0xFF);
    cmdData.push_back((offset >> 8) & 0xFF);
    cmdData.push_back((offset >> 16) & 0xFF);
    cmdData.push_back((offset >> 24) & 0xFF);
    cmdData.push_back((sendData.size()) & 0xFF);
    cmdData.push_back((sendData.size() >> 8) & 0xFF);*/
    cmdData.insert(cmdData.end(), {bios_update, (offset) & 0xFF, (offset >> 8) & 0xFF, (offset >> 16) & 0xFF, (offset >> 24) & 0xFF,(sendData.size()) & 0xFF, (sendData.size() >> 8) & 0xFF});
    cmdData.insert(cmdData.end(), std::move_iterator(sendData.begin()),
      							   std::move_iterator(sendData.end()));
	//total_len = cmdData.size();
    //cmdData.insert(cmdData.begin(), total_len);
	
	//std::cerr << "send_data values : 0x";
    //std::copy( cmdData.begin(), cmdData.end(), std::ostream_iterator<int>( std::cerr << std::hex, " 0x" ) );
    //std::cerr << "send_data size " << cmdData.size() << "\n";
	//const auto hex_str = uint8_vector_to_hex_string(cmdData);
	//std::cerr << "send_data values= " << hex_str << "\n";

        sendIPMBRequest(slot_id, net_fun, cmd_id, cmdData, respData);
		if (respData[0] != 0)
		{
			std::cerr << "F " << offset << " " ;
		}

		//std::cerr << "respDataSize = " << respData.size() << " " << offset << "\n";
		//std::cerr << "respData = " << +respData[0] << " " << +respData[1] << " " << +respData[2] << "\n "; 
#if 0
    while (retries != 0)
    {
//	std::vector<uint8_t> cmdDataa{0x15, 0xa0, 0};
//        sendIPMBRequest(0, net_fun, 3, cmdDataa, respData);
        sendIPMBRequest(slot_id, net_fun, cmd_id, cmdData, respData);
		std::cerr << "respDataSize = " << respData.size() << "\n";
		std::cerr << "respData = " << +respData[0] << " " << +respData[1] << " " << +respData[2] << "\n "; 
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
		
		retries --;
    }
	
	/*if (retries == 0)
	{
		std::cerr << "Error!!! Not able to send bios data!!! \n"; 
		return -1;
	}*/

#endif
    return 0;
}

int get_chksum_fw(uint8_t slot_id, uint8_t net_fun, uint32_t cmd_get_fw_chksum, uint32_t offset, uint32_t len, std::vector<uint8_t> &respData)
{
    std::vector<uint8_t> cmdData{0x15, 0xa0, 0};
    int retries = 1;
    //int retries = 3;
    int count =12;
    int bios_update=0;
    // Fill the component for which firmware is requested
    /*cmdData.push_back(slot_id);
    cmdData.push_back((offset) & 0xFF);
    cmdData.push_back((offset >> 8) & 0xFF);
    cmdData.push_back((offset >> 16) & 0xFF);
    cmdData.push_back((offset >> 24) & 0xFF);
    cmdData.push_back(len & 0xFF);
    cmdData.push_back((len >> 8) & 0xFF);
    cmdData.push_back((len >> 16) & 0xFF);
    cmdData.push_back((len >> 24) & 0xFF);*/
    cmdData.insert(cmdData.end(), {bios_update, (offset) & 0xFF, (offset >> 8) & 0xFF, (offset >> 16) & 0xFF, (offset >> 24) & 0xFF, len & 0xFF, (len >> 8) & 0xFF, (len >> 16) & 0xFF, (len >> 24) & 0xFF});

        sendIPMBRequest(slot_id, net_fun, cmd_get_fw_chksum, cmdData, respData);
	//std::cerr << "chksum resp: " << respData.size() << "\n";
    /*while (retries !=0)
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
    }*/

    return 0;
}

int me_recovery(uint8_t slot_id, uint8_t net_fun, uint8_t cmd_me, uint8_t mode) //RECOVERY_MODE=mode(0x1) cmd_me=0x2
{
    std::vector<uint8_t> cmdData{0x15, 0xa0, 0};
    std::vector<uint8_t> respData;
	uint8_t bic_intf_me = 0x1;    
    int retries = 3;

	cmdData.push_back(bic_intf_me);
	cmdData.push_back(0xB8);
	cmdData.push_back(0xDF);
	cmdData.push_back(0x57);
	cmdData.push_back(0x01);
	cmdData.push_back(0x00);
	cmdData.push_back(mode);

    while (retries != 0)
    {
	    sendIPMBRequest(slot_id, net_fun, cmd_me, cmdData, respData);
        std::cerr << "Me: respData.size " << respData.size() << "\n";
		if (respData.size() !=6)		
		{
	    	sleep(1);
			std::cerr << "ME is not set into recovery mode.. Retrying... \n";
		}
		else if (respData[3] != cmdData[3])
		{
	    	sleep(1);
			std::cerr << "Interface not valid.. Retrying...  \n";
		}
		else if (respData[0] == 0)
		{
	    	sleep(1);
			std::cerr << "ME recovery mode -> Completion Status set.. \n";
			break;
		}
		else if (respData[0] != 0)
		{
	    	sleep(1);
			std::cerr << "ME recovery mode -> Completion Status not set.. Retrying..\n";
		}
		else
		{
	    	sleep(1);
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
    std::vector<uint8_t> meData{0x15, 0xa0, 0};
    std::vector<uint8_t> meResp;
	meData.push_back(bic_intf_me);
	meData.push_back(0x18);
	meData.push_back(0x04);
	retries = 3;
	
    while (retries != 0)
    {
	    sendIPMBRequest(slot_id, net_fun, cmd_me, meData, meResp);
		if (meResp[3] != meData[3])
		{
	    	sleep(1);
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

int bios_update(uint8_t slot_id, const char* image_path)
{

    std::cerr << "Bios update bin file path " << image_path <<"\n";

    // Read the binary data from bin file
    int count	= 0;
    int cmd_id 	= 9;
    uint8_t net_fun = 0x38;
    uint32_t offset = 0;
    uint32_t ipmb_write_max = 222; //224;
	uint8_t cmd_me = 0x2;
	uint8_t recovery_mode =0x1;

	
	// Set ME to recovery mode
	/*int ret_val = me_recovery(slot_id, net_fun, cmd_me, recovery_mode);
	if (ret_val != 0)
	{
		std::cerr << "Me set to recovery mode failed\n";
		return -1;
	}*/

    // Open the file
    std::streampos fileSize;
    std::ifstream file(image_path, std::ios::in|std::ios::binary|std::ios::ate);
    
    if (file.is_open())
    {   
    
        // Get its size
        fileSize = file.tellg();
        std::cerr << "Filesize " << fileSize <<"\n";
        file.seekg(0, std::ios::beg);
 		int i = 0;

        while (offset <= fileSize) 
        //while (offset <= 1000) 
        {
			// Read the data
            std::vector<uint8_t> fileData(ipmb_write_max);
            file.read((char*) &fileData[0], ipmb_write_max);

            //std::cerr << "Offset " << offset <<"\n";
			//std::cerr << "Bios Image value= ";
            //std::copy( fileData.begin(), fileData.end(), std::ostream_iterator<int>( std::cerr << std::hex, " " ) );
            //std::cerr << "Size " << fileData.size() << "\n";
			//const auto hex_str = uint8_vector_to_hex_string(fileData);
			//std::cerr << "Bios image values= " << hex_str << "\n";


        	int ret = send_data(slot_id, net_fun, cmd_id, fileData, offset);
        	/*int ret = send_data(slot_id, net_fun, cmd_id, fileData, offset);
			if (ret != 0)
			{
				return -1;
			}*/
			if (offset == (i *(100*222)))
			{
				std::cerr << ".";
				i=i+1;
			}
			
            offset += ipmb_write_max;
        }


        // Check for bios image
        uint32_t offset_d = 0;
        file.seekg(0, std::ios::beg);
        uint32_t bios_verify_pkt_size = (32*1024);
        std::cerr << "Verify Bios image...\n";

        while (offset_d <= fileSize)
        {
            // Read the data
            std::vector<int> chksum(bios_verify_pkt_size);
            file.read((char*) &chksum[0], bios_verify_pkt_size);
            //std::cerr << "Offset_d " << offset_d << "\n";

            uint32_t tcksum = 0;
            for (int i = 0; i < chksum.size(); i++)
            {
                tcksum += chksum[i];
            }
            //std::cerr << "tcksum = " << tcksum << "\n";
	    
			std::vector<std::uint8_t> calChksum((std::uint8_t*)&tcksum
                                   , (std::uint8_t*)&(tcksum) + sizeof(std::uint32_t));

	
			std::cerr << "tcksum values : 0x";
    		std::copy( calChksum.begin(), calChksum.end(), std::ostream_iterator<int>( std::cerr << std::hex, " 0x" ) );
		    std::cerr << "\n";

	    	// Get the checksum value from firmware
	    	uint8_t retValue;
	    	uint32_t cmd_get_fw_chksum = 0xA;
	   		std::vector<uint8_t> fwChksumData;
	    	
			retValue = get_chksum_fw(slot_id, net_fun, cmd_get_fw_chksum, offset_d, bios_verify_pkt_size, fwChksumData);
			/*if (retValue != 0)
			{
				std::cerr << "Failed to get the Checksum value!! \n";
				return -1;
			}*/


			std::cerr << "fwChksumData values : 0x";
    		std::copy( fwChksumData.begin(), fwChksumData.end(), std::ostream_iterator<int>( std::cerr << std::hex, " 0x" ) );
		    std::cerr << "\n";

			/*for (uint8_t i=0; i <= calChksum.size(); i++)
			{
    	    	// Compare both and see if they match or not
	    	    if (fwChksumData[i] != calChksum[i]) 
		    	{
      				std::cerr <<"checksum does not match, offset:" << offset_d << " Calculated chksum:" << +calChksum[i] << " FW Chksum:" << +fwChksumData[i] <<"\n";
      				//return -1;
	    		}
			} */   	    
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
	
	struct timeval tp1;
    gettimeofday(&tp1, NULL);

    // Get the bin file
    const char* bin_file = argv[1];
    //const char * slot_id = argv[1];
    uint8_t slot_id = 0; //(int) (argv[2]);
	
    std::cerr << "Bin File Path " << bin_file << "\n";
    std::cerr << "IPMB Based Bios Upgrade Started for slot#" << +slot_id << "\n";
    int ret = bios_update(slot_id, bin_file);
    if (ret != 0) 
    {
	std::cerr << "IPMB Based Bios upgrade failed for slot#" << +slot_id << "\n";
	return -1;
    }

    std::cerr << "IPMB Based Bios upgrade completed successfully for slot#" << +slot_id << "\n";
	
	struct timeval tp2;
    gettimeofday(&tp2, NULL);
    long long mslong1 = (long long) tp1.tv_sec * 1000L + tp1.tv_usec / 1000; //get current timestamp in milliseconds
    long long mslong2 = (long long) tp2.tv_sec * 1000L + tp2.tv_usec / 1000; //get current timestamp in milliseconds
    std::cout << "Time start in ms " << mslong1 << std::endl;
    std::cout << "Time stop in ms " << mslong2 << std::endl;

	io.run();
    return 0;
}

