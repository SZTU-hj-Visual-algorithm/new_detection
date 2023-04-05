#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

//协方差矩阵，用于里程计话题数据，用于robt_pose_ekf功能包

#define HEADER_SOF 0xA5
#define END1_SOF 0x0D
#define END2_SOF 0x0A

#pragma pack(push, 1)

typedef enum
{
  CHASSIS_ODOM_CMD_ID = 0x0101,
  CHASSIS_CTRL_CMD_ID = 0x0102,
  RGB_ID=0x0103,
  RC_ID=0x0104,
  VISION_ID=0x0105
} referee_data_cmd_id_type;

typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

typedef struct
{
    float vx=0;
    float vy=0;
    float vw=0;
    float yaw=0;
    float pitch=0;
    int8_t target_lock=0;
    int8_t fire_command;
}robot_ctrl_info_t;

typedef struct
{
  uint16_t id;
  uint16_t mode;
  float pitch=0;
  float yaw=0;
  float roll=0;
  float quaternion[4]={0};
  float shoot=0;
} vision_t;

#pragma pack(pop)
#endif //ROBOMASTER_PROTOCOL_H
