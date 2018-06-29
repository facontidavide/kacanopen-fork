/*
 * Copyright (c) 2015-2016, Thomas Keh
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
#include <chrono>
#include <vector>
#include <iostream>
#include "core.h"

int32_t parse_sdo_read(std::vector<uint8_t> sdo_to_read){

		unsigned char sdo_read_byte[]={0x00,0x00,0x00,0x00};

		for (unsigned int i=0; i<sdo_to_read.size();i++) {
			sdo_read_byte[i] = sdo_to_read.at(i);
			//std::cout<<"sdo_to_read["<<i<<"] 0x"<< std::hex << (unsigned) sdo_read_byte[i] << std::endl;
		}
		int16_t sdo_read_w0 = (sdo_read_byte[1] << 8) + sdo_read_byte[0];
		int16_t sdo_read_w1 = (sdo_read_byte[3] << 8) + sdo_read_byte[2];
		int32_t sdo_read_dw = (sdo_read_w1 << 16) + sdo_read_w0;
		//std::cout<<"size of the sdo read="<<sdo_to_read.size()<<std::endl;
		if (2<sdo_to_read.size()){
			
			//std::cout<<"I am returing int32"<<std::endl;
			return sdo_read_dw;
		} else{
			short int ret = sdo_read_dw;
			//std::cout<<"I am returing short int"<<std::endl;
			return ret;	
		}
		

	}

int main() {

	// ----------- //
	// Preferences //
	// ----------- //


	//A Roboteq motor driver was used to test this program //



	// The node ID of the slave we want to communicate with.
	const uint8_t node_id = 4;

	// Set the name of your CAN bus. "slcan0" is a common bus name
	// for the first SocketCAN device on a Linux system.
	const std::string busname = "slcan0";

	// Set the baudrate of your CAN bus. Most drivers support the values
	// "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
	const std::string baudrate = "500K";

	// Set the object dictionary index to write to (download).
	// Here: CiA-401 (I/O device) digital output.

	const uint16_t index_digtal_out_write = 0x2008;

	// Alternative: CiA-402 (motor) control word:
	//const uint16_t index = 0x6040;
	const uint16_t index_ch1_speed = 0x2000;
	const uint16_t index_ch2_speed = 0x2000;
	const uint16_t index_ch1_speed_feedback = 0x2103;
	const uint16_t index_ch2_speed_feedback = 0x2103;
	const uint16_t index_ch1_Battery_Amps = 0x210C;
	const uint16_t index_ch2_Battery_Amps = 0x210C;
	// Set the object dictionary sub-index to write to (download).
	// Here: CiA-401 (I/O device) digital output - second byte.
	const uint8_t subindex_digtal_out_write = 0x00;
	// Alternative: CiA-402 (motor) control word:
	//const uint8_t subindex = 0x00;
	const uint8_t subindex_ch1_speed = 0x01;
	const uint8_t subindex_ch2_speed = 0x02;
	const uint8_t subindex_ch1_speed_feedback = 0x01;
	const uint8_t subindex_ch2_speed_feedback = 0x02;
	const uint8_t subindex_ch1_Battery_Amps = 0x01;
	const uint8_t subindex_ch2_Battery_Amps = 0x02;
	// Set the data to write (download).
	const std::vector<uint8_t> digtal_out_write { 0x2 };
	const std::vector<uint8_t> ch1_speed {0x34,0xF3,0xFF,0xFF}; // speed reference to roboteq : +3276 .This is the lowest valid reference.
	//const std::vector<uint8_t> ch1_speed {0x34,0xF3,0xFF,0xFF}; //speed reference to roboteq :-3276 .This is the lowest valid reference.
	const std::vector<uint8_t> ch2_speed {0xCC,0x0C,0x00,0x00}; // speed reference to roboteq : +3276 .This is the lowest valid reference.
	//const std::vector<uint8_t> ch2_speed {0x34,0xF3,0xFF,0xFF}; //speed reference to roboteq :-3276 .This is the lowest valid reference.
	
	// Alternative: CiA-402 (motor) control word has two bytes. Command: shutdown (little-endian!)
	//const std::vector<uint8_t> data { 0x06, 0x00 };

	// -------------- //
	// Initialization //
	// -------------- //



	std::cout << "This is an example which shows the usage of the Core library." << std::endl;

	// Create core.
	kaco::Core core;
	// This will be set to true by the callback below.
	bool found_node = false;

	std::cout << "Registering a callback which is called when a device is detected via NMT..." << std::endl;
	core.nmt.register_device_alive_callback([&] (const uint8_t new_node_id) {
		std::cout << "Device says it's alive! ID = " << (unsigned) new_node_id << std::endl;
		// Check if this is the node we are looking for.
		if (new_node_id == node_id) {
			found_node = true;
		}
	});
	
	std::cout << "Starting Core (connect to the driver and start the receiver thread)..." << std::endl;
	if (!core.start(busname, baudrate)) {
		std::cout << "Starting core failed." << std::endl;
		return EXIT_FAILURE;
	}

	//std::cout << "Asking all devices to reset. You don't need to do that, but it makes"
	//	<< " sure all slaves are in a reproducible state." << std::endl;
	//core.nmt.reset_all_nodes();

	// As an alternative you can request the slaves to announce
	// themselves:
	// core.nmt.discover_nodes();

	std::cout << "Giving the devices one second time to respond..." << std::endl;
	std::this_thread::sleep_for(std::chrono::seconds(1));

	// Check if the device has been detected (see the callback above).
	if (!found_node) {
		std::cout << "Node with ID " << (unsigned) node_id << " has not been found."<< std::endl;
		return EXIT_FAILURE;
	}

	std::cout << "Asking the device to start up..." << std::endl;
	core.nmt.send_nmt_message(node_id,kaco::NMT::Command::start_node);

	std::cout << "Giving the devices one second time to boot up..." << std::endl;
	std::this_thread::sleep_for(std::chrono::seconds(1));
	if (found_node){
		std::vector<uint8_t> read_device_type = core.sdo.upload(node_id,0x1000,0x0);
		std::vector<uint8_t> read_device_name = core.sdo.upload(node_id,0x1008,0x0);
		std::vector<uint8_t> read_hardware_version = core.sdo.upload(node_id,0x1009,0x0);
		std::vector<uint8_t> read_firmware_version = core.sdo.upload(node_id,0x100A,0x0);
		uint16_t profile = parse_sdo_read(read_device_type);
		std::cout << ""<<std::endl;
		std::cout << ""<<std::endl;
		std::cout << "***************************************************************************"<<std::endl;
		std::cout << "***************************************************************************"<<std::endl;
		std::cout << "* Device Profile number found CiA-"<<std::dec<<profile<<""<<std::endl;
		std::cout << "* Device Name found as '";
		std::string device_name(reinterpret_cast<char const*>(read_device_name.data()), read_device_name.size());
		std::cout << device_name << "'" << std::endl;
		std::cout << "* Hardware version=";
		std::string hardware_version(reinterpret_cast<char const*>(read_hardware_version.data()), read_hardware_version.size());
		std::cout << hardware_version <<"" <<std::endl;
		std::cout << "* Firmware version=";
		std::string firmware_version(reinterpret_cast<char const*>(read_firmware_version.data()), read_firmware_version.size());
		std::cout << hardware_version <<"" <<std::endl;
		std::cout<<"***************************************************************************"<<std::endl;
		std::cout<<"***************************************************************************"<<std::endl;
		std::cout<<""<<std::endl;
		std::cout<<""<<std::endl;

	}
	std::cout << "Writing digital out to "<< std::hex << index_digtal_out_write <<std::endl;
	core.sdo.download(node_id, index_digtal_out_write, subindex_digtal_out_write, digtal_out_write.size(), digtal_out_write);
	std::cout << "Waiting for a second..." << std::endl;
	std::this_thread::sleep_for(std::chrono::seconds(1));
	// ------------ //
	// Device usage //
	// ------------ //
	while (found_node){
		
		
		
		std::cout << "Sending speed reference to Roboteq channel 1..." << std::endl;
		core.sdo.download(node_id, index_ch1_speed, subindex_ch1_speed, ch1_speed.size(), ch1_speed);
		std::cout << "Sending speed reference to Roboteq channel 2..." << std::endl;
		core.sdo.download(node_id, index_ch2_speed, subindex_ch2_speed, ch2_speed.size(), ch2_speed);	
		std::cout << "Reading the sdo at "<< std::hex<< index_ch1_speed_feedback <<"...." << std::endl;
		std::vector<uint8_t> ch1_speed_feedback = core.sdo.upload(node_id,index_ch1_speed_feedback,subindex_ch1_speed_feedback);
		std::vector<uint8_t> ch2_speed_feedback = core.sdo.upload(node_id,index_ch2_speed_feedback,subindex_ch2_speed_feedback);
		std::vector<uint8_t> ch1_Battery_Amps = core.sdo.upload(node_id,index_ch1_Battery_Amps,subindex_ch1_Battery_Amps);
		std::vector<uint8_t> ch2_Battery_Amps = core.sdo.upload(node_id,index_ch2_Battery_Amps,subindex_ch2_Battery_Amps);
		int readout1= parse_sdo_read(ch1_speed_feedback);
		int readout2= parse_sdo_read(ch2_speed_feedback);
		int readout3= parse_sdo_read(ch1_Battery_Amps);
		int readout4= parse_sdo_read(ch2_Battery_Amps);
		std::cout <<"Channel 1 speed in hex= 0x" << std::hex << readout1 << std::endl;
		std::cout <<"Channel 2 speed in decimal= " << std::dec << readout2 << std::endl;
		std::cout <<"Channel 1 Battery Amps in decimal=" << std::dec << readout3 << std::endl;
		std::cout <<"Channel 2 Battery Amps in decimal=" << std::dec << readout4 << std::endl;



	}
	std::cout << "Finished." << std::endl;
	return EXIT_SUCCESS;

}
