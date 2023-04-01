#include "serial_main.h"

SerialMain::SerialMain(std::string device_path) : device_path_(device_path)
{
	if (!(CommInit()))
	{
		std::cout<<"serial init error!!!!!!!!!!"<<std::endl;
	};
}

void SerialMain::SenderMain(const std::vector<double> &vdata)
{
	robot_ctrl.pitch = vdata[0];
	robot_ctrl.yaw = vdata[1];
	robot_ctrl.target_lock = vdata[2];
	uint16_t send_length = SenderPackSolve((uint8_t *)&robot_ctrl, sizeof(robot_ctrl_info_t),
										   CHASSIS_CTRL_CMD_ID, send_buff_.get());
	device_ptr_->Write(send_buff_.get(), send_length);
	
}

bool SerialMain::CommInit()
{
	
	device_ptr_ = std::make_shared<SerialDevice>(device_path_, 115200); // 比特率115200
	
	if (!device_ptr_->Init())
	{
		return false;
	}
	
	recv_buff_ = std::unique_ptr<uint8_t[]>(new uint8_t[BUFF_LENGTH]);
	send_buff_ = std::unique_ptr<uint8_t[]>(new uint8_t[BUFF_LENGTH]);
	
	memset(&frame_receive_header_, 0, sizeof(frame_header_struct_t));
	memset(&frame_send_header_, 0, sizeof(frame_header_struct_t));
	
	return true;
}

bool SerialMain::ReceiverMain()
{
	
	int a = 0;
	int flag = 0;
	bool get = false;
	uint8_t last_len = 0;
	int count = 2;
	while (count--)
	{
		// uint16_t read_length = device_ptr_->Read(recv_buff_.get(),BUFF_LENGTH);
		
		last_len = device_ptr_->ReadUntil2(recv_buff_.get(), END1_SOF, END2_SOF, 128);
		
		while (flag == 0 && last_len == 1)
		{
			if ((recv_buff_[a] == END1_SOF) && (recv_buff_[a + 1] == END2_SOF))
			{
				flag = 1;
				SearchFrameSOF(recv_buff_.get(), a);
				get = true;
			}
			// printf("%x  ",recv_buff_[a]);
			a++;
		}
		flag = 0;
		a = 0;
	}
	return get;
}

void SerialMain::SearchFrameSOF(uint8_t *frame, uint16_t total_len)
{
	uint16_t i;
	uint16_t index = 0;
	int a = 0;
	
//	std::cout<<total_len<<std::endl;
	for (i = 0; i < total_len;)
	{
		if (*frame == HEADER_SOF)
		{
			ReceiveDataSolve(frame);
			i = total_len;
		}
		else
		{
			frame++;
			i++;
		}
	}
}

uint16_t SerialMain::ReceiveDataSolve(uint8_t *frame)
{
	uint8_t index = 0;
	uint16_t cmd_id = 0;
	
	if (*frame != HEADER_SOF)
	{
		return 0;
	}
	
	memcpy(&frame_receive_header_, frame, sizeof(frame_header_struct_t));
	index += sizeof(frame_header_struct_t);
	
	if ((!Verify_CRC8_Check_Sum(frame, sizeof(frame_header_struct_t))) || (!Verify_CRC16_Check_Sum(frame, frame_receive_header_.data_length + 9)))
	{
		std::cout<<"CRC error!!"<<std::endl;
		return 0;
	}
	else
	{
		memcpy(&cmd_id, frame + index, sizeof(uint16_t));
		index += sizeof(uint16_t);
		// printf("id:%x\n", cmd_id);
		switch (cmd_id)
		{
			case VISION_ID:
			{
				memcpy(&vision_msg_, frame + index, sizeof(vision_t));
                //---------------------serial_main  data------------
//                std::cout<<"-----serial_main  data------"<<std::endl;
//				std::cout<<"mode:"<<vision_msg_.mode<<std::endl;
//				std::cout<<"yaw:"<<vision_msg_.yaw<<std::endl;
//				std::cout<<"pitch:"<<vision_msg_.pitch<<std::endl;
//				std::cout<<"quat0:"<<vision_msg_.quaternion[0]<<std::endl;
//				std::cout<<"quat1:"<<vision_msg_.quaternion[1]<<std::endl;
//				std::cout<<"quat2:"<<vision_msg_.quaternion[2]<<std::endl;
//				std::cout<<"quat3:"<<vision_msg_.quaternion[3]<<std::endl;
			}
				break;
			default:
				break;
		}
		index += frame_receive_header_.data_length + 2;
		return index;
	}
}

uint16_t SerialMain::SenderPackSolve(uint8_t *data, uint16_t data_length,
									 uint16_t cmd_id, uint8_t *send_buf)
{
	
	uint8_t index = 0;
	frame_send_header_.SOF = HEADER_SOF;
	frame_send_header_.data_length = data_length;
	frame_send_header_.seq++;
	
	Append_CRC8_Check_Sum((uint8_t *)&frame_send_header_, sizeof(frame_header_struct_t));
	
	memcpy(send_buf, &frame_send_header_, sizeof(frame_header_struct_t));//assign frame header
	
	index += sizeof(frame_header_struct_t);
	
	memcpy(send_buf + index, &cmd_id, sizeof(uint16_t));//assign cmd
	
	index += sizeof(uint16_t);
	
	memcpy(send_buf + index, data, data_length);//assign data
	
	Append_CRC16_Check_Sum(send_buf, data_length + 9);
	
	return data_length + 9;
}