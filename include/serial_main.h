#ifndef ROBOMASTER_ROBOT_H
#define ROBOMASTER_ROBOT_H

#include "serial_device.h"
#include "protocol.h"
#include "crc.h"
#include<iostream>
#include <thread>
#include <vector>

class SerialMain {
public:
	SerialMain(std::string device_path = "/dev/robomaster");
	
	~SerialMain() = default;
	
	void SenderMain(const std::vector<double> &vdata);
	
	bool CommInit();
	
	bool ReceiverMain();
	
	void SearchFrameSOF(uint8_t *frame, uint16_t total_len);
	
	uint16_t ReceiveDataSolve(uint8_t *frame);
	
	uint16_t SenderPackSolve(uint8_t *data, uint16_t data_length,
							 uint16_t cmd_id, uint8_t *send_buf);
	vision_t vision_msg_;

private:
	
	//! Device Information and Buffer Allocation
	std::string device_path_;
	std::shared_ptr<SerialDevice> device_ptr_;
	std::unique_ptr<uint8_t[]> recv_buff_;
	std::unique_ptr<uint8_t[]> send_buff_;
	const unsigned int BUFF_LENGTH = 512;
	
	//! Frame Information
	frame_header_struct_t frame_receive_header_;
	frame_header_struct_t frame_send_header_;
	
	/** @brief specific protocol data are defined here
	 *         xxxx_info_t is defined in protocol.h
	 */
	
	robot_ctrl_info_t robot_ctrl;
};
//}

#endif // ROBOMASTER_ROBOT_H
