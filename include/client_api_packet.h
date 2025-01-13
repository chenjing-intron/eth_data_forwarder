#ifndef __CLIENT_API_PACKET_H__
#define __CLIENT_API_PACKET_H__

//#include "arm_control_api.h"

#define uint32  unsigned int
#define uint16  unsigned short
#define uint8   unsigned char

#define SERVER_PORT_NUM (7272)

typedef struct __attribute__((packed)) arm_control_request {
    uint16  magic; 
    uint16  length;
    uint16  command;
    uint32  cmd_index;
    uint32  action_index;
    uint32  execute_time_us;
    uint16  gripper_state;
    uint16  checksum;  
} arm_control_request_t;

#define CMD_REQ_DATA_LENGTH (sizeof(arm_control_request_t) - 6)


typedef struct __attribute__((packed)) arm_control_response {
    uint16  magic; 
    uint16  length;
    uint16  command;
    uint16  command_result;
    uint16  checksum;  
} arm_control_response_t;

#define CMD_RESP_DATA_LENGTH (sizeof(arm_control_response_t) - 6)

#define AGV_CAN_GROUP_ID (0xE001)

#define CLIENT_CMD_REGISTER_GROUP  (0xA001)
#define CLIENT_CMD_TRANSFER_DATA   (0xA002)

typedef struct __attribute__((packed)) client_register_request {
    uint16  magic; 
    uint16  length;
    uint16  command;
    uint16  group_id;
    uint16  checksum;  
} client_register_request_t;

#define CLIENT_REGISTER_DATA_LENGTH (sizeof(client_register_request_t) - 6)



typedef struct __attribute__((packed)) client_data {
    uint16  command;
    uint8   data[0];
} client_data_t;


#define MAGIC1 (0x55)
#define MAGIC2 (0xAA)
#define MAGIC_WHOLE (0xAA55)

#define RECV_MAGIC      (0)
#define RECV_LEN        (1)
#define RECV_DATA       (2)
#define RECV_CHECKSUM   (3)

#define RECV_STATE_MAX  (4)

#define FIELD_MAGIC     (0)
#define FIELD_LENGTH    (1)
#define FIELD_DATA      (2)
#define FIELD_CHECKSUM  (3)

#define FIELD_NUM (4)

#define FIELD_MAGIC_LEN       (2)
#define FIELD_LENGTH_LEN      (2)
#define FIELD_CHECKSUM_LEN    (2)
#define MAX_COMMAND_DATA_LEN  (120)

typedef struct field_data
{
   int max_len;
   int actual_len;
   int cur_len;
   uint8_t *data;
} field_data_t;

#endif