#ifndef __ARM_CONTROL_PACKET_H__
#define __ARM_CONTROL_PACKET_H__

//#include "arm_control_api.h"

#define uint32  unsigned int
#define uint16  unsigned short
#define uint8   unsigned char

#define MAX_JOINT_NUM (6)

#define DEG_CONVERSION_PARAM (4599.0f)

#define CMD_GET_ARM_STATUS           (0)
#define CMD_SET_JOINT_POS            (1)
#define CMD_GET_JOINT_POS            (2)
#define CMD_SET_GRIPPER_STATE        (3)
#define CMD_GET_GRIPPER_STATE        (4)
#define CMD_GET_JOINT_ACTION_STATUS  (5)

typedef struct __attribute__((packed)) arm_control_request {
    uint16  magic; 
    uint16  length;
    uint16  command;
    uint32  cmd_index;
    uint32  action_index;
    uint32  execute_time_us;
    uint32  joint_pos[MAX_JOINT_NUM];
    uint16  gripper_state;
    uint16  checksum;  
} arm_control_request_t;

#define CMD_REQ_DATA_LENGTH (sizeof(arm_control_request_t) - 6)


typedef struct __attribute__((packed)) arm_control_response {
    uint16  magic; 
    uint16  length;
    uint16  command;
    uint16  command_result;
    uint32  cmd_index;
    uint32  action_index;
    uint32  joint_pos[MAX_JOINT_NUM];
    uint16  gripper_state;
    uint16  checksum;  
} arm_control_response_t;

#define CMD_RESP_DATA_LENGTH (sizeof(arm_control_response_t) - 6)

#define MAGIC1 (0x55)
#define MAGIC2 (0xAA)
#define MAGIC_WHOLE (0xAA55)

#define RECV_MAGIC (0)
#define RECV_LEN (1)
#define RECV_DATA (2)
#define RECV_CHECKSUM (3)
#define RECV_STATE_MAX (4)

#define FIELD_MAGIC (0)
#define FIELD_LENGTH (1)
#define FIELD_DATA (2)
#define FIELD_CHECKSUM (3)

#define FIELD_NUM (4)

#define FIELD_MAGIC_LEN (2)
#define FIELD_LENGTH_LEN (2)
#define FIELD_CHECKSUM_LEN (2)
#define MAX_COMMAND_DATA_LEN (100)

typedef struct field_data
{
   int max_len;
   int actual_len;
   int cur_len;
   uint8_t *data;
} field_data_t;

#endif