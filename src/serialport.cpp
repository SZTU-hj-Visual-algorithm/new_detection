#include "serialport.h"
using namespace std;

/**
 * @brief init action
 */

SerialPort::SerialPort()
{
	initSerialPort();
}

/**
 *@brief   获取模式命令
 */

bool SerialPort::get_Mode1(int &mode, float &pitch, float &yaw, float &roll, float &ball_speed, int &color)
{
	int bytes;
	//!< check whether open serial valid
	char *name = ttyname(fd);
	if (!name) printf("tty is null\n");
	//printf("device:%s\n",name);
	
	int result = ioctl(fd, FIONREAD, &bytes);
	if (result == -1)
	{
//        fd = open("/dev/ttyUSB", O_RDWR | O_NOCTTY | O_NDELAY);
//
//        speed = BAUDRATE;
//        databits = 8;
//        stopbits = 1;
//        parity = 'N';
//        initSerialPort();
		return false;
	}
	cout<<bytes<<endl;
	if (bytes == 0)
	{
        fmt::print("缓冲区为空");
		return true;
	}
	
	bytes = read(fd, rdata, 44);
	cout<<bytes<<endl;
	int i=0;
	for(;i<44;i++)
	{
		printf("rdata[i]:%x\n",rdata[i]);
		if(rdata[i] == 0xA5)
		{
			if(Verify_CRC8_Check_Sum(rdata+i, 3))
			{
				mode  = (int)rdata[1+i];
				tcflush(fd, TCIFLUSH);
				if(Verify_CRC16_Check_Sum(rdata+i,22))
				{
					//            printf("1111");
					VisionData demo;
					demo.pitch_angle.c[0] = rdata[3+i];
					demo.pitch_angle.c[1] = rdata[4+i];
					demo.pitch_angle.c[2] = rdata[5+i];
					demo.pitch_angle.c[3] = rdata[6+i];
					pitch = demo.pitch_angle.f;
					
					demo.yaw_angle.c[0] = rdata[7+i];
					demo.yaw_angle.c[1] = rdata[8+i];
					demo.yaw_angle.c[2] = rdata[9+i];
					demo.yaw_angle.c[3] = rdata[10+i];
					yaw = demo.yaw_angle.f;
					
					demo.yaw_angle.c[0] = rdata[11+i];
					demo.yaw_angle.c[1] = rdata[12+i];
					demo.yaw_angle.c[2] = rdata[13+i];
					demo.yaw_angle.c[3] = rdata[14+i];
					roll = demo.yaw_angle.f;
					
					demo.yaw_angle.c[0] = rdata[15+i];
					demo.yaw_angle.c[1] = rdata[16+i];
					demo.yaw_angle.c[2] = rdata[17+i];
					demo.yaw_angle.c[3] = rdata[18+i];
					ball_speed = demo.yaw_angle.f;
					
					color = (int)rdata[19+i];
					ball_speed = 28.0;
					
					//          printf("//////port//////\n");
					//		    printf("pitch :%f\n",pitch);
					//		    printf("yaw   :%f\n",yaw);
					//		    printf("roll  :%f\n",roll);
					//          printf("mode  :%x\n",mode);
					//		    printf("speed :%f\n",ball_speed);
					//          printf("color :%d\n",color);
					//          printf("////////////////\n");
					return true;
				}
				else
				{
					printf("error:CRC16\n");
					return false;
				}
			}
		}
	}
	tcflush(fd, TCIFLUSH);
	printf("error:CRC8 or A5\n");
	return false;
}
/**
 *@brief   新的串口数据接收格式
 */
bool SerialPort::get_Mode1_new(int &mode, float &pitch, float &yaw, float &ball_speed, float *quaternion)
{
	//!< check whether open serial valid
	char *name = ttyname(fd);
	if (!name){
		initSerialPort();
		printf("tty is null\n");
		return false;
	}
	//printf("device:%s\n",name);
	
	int bytes;
	int result = ioctl(fd, FIONREAD, &bytes);
	if (result == -1)
	{
		return false;
	}
	
	if (bytes == 0)
	{
        fmt::print("缓冲区为空");
		return true;
	}
	read(fd, rdata, RECEIVE_LENGTH*2);
	for(int i=0;i<RECEIVE_LENGTH*2;i++)
	{
		if(rdata[i] == 0xA5 && Verify_CRC8_Check_Sum(rdata+i, 3))
		{
			mode  = (int)rdata[1+i];
			tcflush(fd, TCIFLUSH);
			if(Verify_CRC16_Check_Sum(rdata+i,RECEIVE_LENGTH))
			{
				ReceiveData demo;
				demo.CacheData.c[0] = rdata[3+i];
				demo.CacheData.c[1] = rdata[4+i];
				demo.CacheData.c[2] = rdata[5+i];
				demo.CacheData.c[3] = rdata[6+i];
				pitch = demo.CacheData.f;
				
				demo.CacheData.c[0] = rdata[7+i];
				demo.CacheData.c[1] = rdata[8+i];
				demo.CacheData.c[2] = rdata[9+i];
				demo.CacheData.c[3] = rdata[10+i];
				yaw = demo.CacheData.f;
				
				demo.CacheData.c[0] = rdata[11+i];
				demo.CacheData.c[1] = rdata[12+i];
				demo.CacheData.c[2] = rdata[13+i];
				demo.CacheData.c[3] = rdata[14+i];
				ball_speed = demo.CacheData.f;
				
				demo.CacheData.c[0] = rdata[15+i];
				demo.CacheData.c[1] = rdata[16+i];
				demo.CacheData.c[2] = rdata[17+i];
				demo.CacheData.c[3] = rdata[18+i];
				quaternion[0] = demo.CacheData.f;
				
				demo.CacheData.c[0] = rdata[19+i];
				demo.CacheData.c[1] = rdata[20+i];
				demo.CacheData.c[2] = rdata[21+i];
				demo.CacheData.c[3] = rdata[22+i];
				quaternion[1] = demo.CacheData.f;
				
				demo.CacheData.c[0] = rdata[23+i];
				demo.CacheData.c[1] = rdata[24+i];
				demo.CacheData.c[2] = rdata[25+i];
				demo.CacheData.c[3] = rdata[26+i];
				quaternion[2] = demo.CacheData.f;
				
				demo.CacheData.c[0] = rdata[27+i];
				demo.CacheData.c[1] = rdata[28+i];
				demo.CacheData.c[2] = rdata[29+i];
				demo.CacheData.c[3] = rdata[30+i];
				quaternion[3] = demo.CacheData.f;
				
				ball_speed = 28.0;

//                printf("get_yaw:%f   |\n",yaw);
//                printf("get_pitch:%f |\n",pitch);
//				printf("get_quaternion0:   |%f\n",quaternion[0]);
//				printf("get_quaternion1:   |%f\n",quaternion[1]);
//				printf("get_quaternion2:   |%f\n",quaternion[2]);
//				printf("get_quaternion3:   |%f\n",quaternion[3]);
				return true;
			}
			else
			{
				printf("error:CRC16\n");
				return false;
			}
		}
	}
	printf("error:CRC8 or A5\n");
	tcflush(fd, TCIFLUSH);
	return false;
}

/**
 *@brief   初始化数据
 *@param  fd       类型  int  打开的串口文件句柄
 *@param  speed    类型  int  波特率
 *@param  databits 类型  int  数据位   取值 为 7 或者8
 *@param  stopbits 类型  int  停止位   取值为 1 或者2
 *@param  parity   类型  int  效验类型 取值为N,E,O,S
 */
bool SerialPort::initSerialPort()
{
	speed = BAUDRATE;
	databits = 8;
	stopbits = 1;
	parity = 'N';
	fd = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
	fd = open(UART_DEVICE, O_RDWR | O_NOCTTY);
	if (fd == -1)
	{
		perror(UART_DEVICE);
		return false;
	}
    fmt::print("Opening...");
	set_Brate();
	
	if (set_Bit() == FALSE)
	{
		printf("Set Parity Error\n");
		exit(0);
	}
	
	//    set_disp_mode(0);
	printf("Open successed\n");
	return true;
}

/**
 *@brief   设置波特率
 */
void SerialPort::set_Brate()
{
	int speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
					   B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
	};
	int name_arr[] = {115200, 38400, 19200, 9600, 4800, 2400, 1200,  300,
					  115200, 38400, 19200, 9600, 4800, 2400, 1200,  300,
	};
	int   i;
	int   status;
	struct termios   Opt;
	tcgetattr(fd, &Opt);
	
	for (i = 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
	{
		if (speed == name_arr[i])
		{
			tcflush(fd, TCIOFLUSH);//清空缓冲区的内容
			cfsetispeed(&Opt, speed_arr[i]);//设置接受和发送的波特率
			cfsetospeed(&Opt, speed_arr[i]);
			status = tcsetattr(fd, TCSANOW, &Opt); //使设置立即生效
			
			if (status != 0)
			{
				perror("tcsetattr fd1");
				return;
			}
			
			tcflush(fd, TCIOFLUSH);
			
		}
	}
}

/**
 *@brief   设置串口数据位，停止位和效验位
 */
int SerialPort::set_Bit()
{
	struct termios termios_p;
	
	if (tcgetattr(fd, &termios_p)  !=  0)
	{
		perror("SetupSerial 1");
		return (FALSE);
	}
	
	termios_p.c_cflag |= (CLOCAL | CREAD);  //接受数据
	termios_p.c_cflag &= ~CSIZE;//设置数据位数
	
	switch (databits)
	{
		case 7:
			termios_p.c_cflag |= CS7;//CS7代表数据宽度是7bit
			break;
		
		case 8:
			termios_p.c_cflag |= CS8;//CS8代表数据宽度为8bit，一般都是用8bit的数据的
			break;
		
		default:
			fprintf(stderr, "Unsupported data size\n");
			return (FALSE);
	}
	
	//设置奇偶校验位double
	switch (parity)
	{
		case 'n':
		case 'N':
			termios_p.c_cflag &= ~PARENB;   /* Clear parity enable */
			termios_p.c_iflag &= ~INPCK;     /* Enable parity checking */
			break;
		
		case 'o':
		case 'O':
			termios_p.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
			termios_p.c_iflag |= INPCK;             /* Disnable parity checking */
			break;
		
		case 'e':
		case 'E':
			termios_p.c_cflag |= PARENB;     /* Enable parity */
			termios_p.c_cflag &= ~PARODD;   /* 转换为偶效验*/
			termios_p.c_iflag |= INPCK;       /* Disnable parity checking */
			break;
		
		case 'S':
		case 's':  /*as no parity*/
			termios_p.c_cflag &= ~PARENB;
			termios_p.c_cflag &= ~CSTOPB;
			break;
		
		default:
			fprintf(stderr, "Unsupported parity\n");
			return (FALSE);
		
	}
	
	/* 设置停止位*/
	switch (stopbits)
	{
		case 1:
			termios_p.c_cflag &= ~CSTOPB;
			break;
		
		case 2:
			termios_p.c_cflag |= CSTOPB;
			break;
		
		default:
			fprintf(stderr, "Unsupported stop bits\n");
			return (FALSE);
		
	}
	
	/* Set input parity option */
	if (parity != 'n')
		termios_p.c_iflag |= INPCK;
	
	tcflush(fd, TCIFLUSH); //清除输入缓存区
	termios_p.c_cc[VTIME] = 150; /* 设置超时15 seconds*/
	termios_p.c_cc[VMIN] = 0;  //最小接收字符
	termios_p.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input原始输入*/
	termios_p.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
	termios_p.c_iflag &= ~(ICRNL | IGNCR);
	termios_p.c_oflag &= ~OPOST;   /*Output禁用输出处理*/
	
	if (tcsetattr(fd, TCSANOW, &termios_p) != 0) /* Update the options and do it NOW */
	{
		perror("SetupSerial 3");
		return (FALSE);
	}
	
	return (TRUE);
}


////////////////////////////////////////////////////////////
/**
 *@brief   转换数据并发送

 *@param  p        类型  int  x坐标
 *@param  yaw      类型  int  y坐标
 *@param  dis      类型  int  dis  距离
 *@param  unsign   类型  int  空白（可添加数据）
 */
void SerialPort::TransformDataFirst(int Xpos, int Ypos, int dis)
{
	
	Tdata[0] = 0xA5;
	Tdata[1] = CmdID0;   //command
	
	Append_CRC8_Check_Sum(Tdata, 3); //CRC8 校验
	
	for (int i = 0; i < 4; i++)
	{
		Tdata[3 + i] = Xpos % 10;
		Xpos = (Xpos - Xpos % 10) / 10;
	}
	
	for (int i = 0; i < 4; i++)
	{
		Tdata[7 + i] = Ypos % 10;
		Ypos = (Ypos - Ypos % 10) / 10;
	}
	
	for (int i = 0; i < 4; i++)
	{
		Tdata[11 + i] = dis % 10;
		dis = (dis - dis % 10) / 10;
	}
	
	Tdata[15] = 0;
	Tdata[16] = 0;
	Tdata[17] = 0;
	Tdata[18] = 0;
	Tdata[19] = 0;
	
	Append_CRC16_Check_Sum(Tdata, 22); //CRC16校验
}


/**
 *@brief   转换数据并发送
 *@param   data   类型  VisionData(union)  包含pitch,yaw,distance
 *@param   flag   类型  char   用于判断是否瞄准目标，0代表没有，1代表已经瞄准
 */
void SerialPort::TransformData(const VisionData &data)
{
	Tdata[0] = 0xA5;
	
	Tdata[1] = data.cmd;
	Append_CRC8_Check_Sum(Tdata, 3);
	
	//printf("cmd::%x\n",Tdata[1]);
	
	Tdata[3] = data.pitch_angle.c[0];
	Tdata[4] = data.pitch_angle.c[1];
	Tdata[5] = data.pitch_angle.c[2];
	Tdata[6] = data.pitch_angle.c[3];
	//printf("pitch:%x%x%x%x\n",Tdata[3],Tdata[4],Tdata[5],Tdata[6]);
	
	Tdata[7] = data.yaw_angle.c[0];
	Tdata[8] = data.yaw_angle.c[1];
	Tdata[9] = data.yaw_angle.c[2];
	Tdata[10] = data.yaw_angle.c[3];
	//printf("yao:%x%x%x%x\n",Tdata[7],Tdata[8],Tdata[9],Tdata[10]);
	
	Tdata[11] = 0x00;
	Tdata[12] = 0x00;
	Tdata[13] = 0x00;
	Tdata[14] = 0x00;
	
	Tdata[15] = 0x00;
	Tdata[16] = 0x00;
	
	Tdata[17] = 0x00;
	Tdata[18] = 0x00;
	Tdata[19] = 0x00;
	
	Append_CRC16_Check_Sum(Tdata, SEND_LENGTH);
	//printf("dis:%x%x\n",Tdata[20],Tdata[21]);
}

//发送数据函数
void SerialPort::send()
{
	write(fd, Tdata, 22);
}

//关闭通讯协议接口
void SerialPort::closePort()
{
	close(fd);
}