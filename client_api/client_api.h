#ifndef __CONTROL_API_H__
#define __CONTROL_API_H__

#define ERR_OK               (0)
#define ERR_NEW_CMD_FAIL     (-1)
#define ERR_SEND_CMD_FAIL    (-2)
#define ERR_RECV_CMD_TIMEOUT (-3)
#define ERR_RECV_CMD_FAILED  (-4)

#define AGV_CAN_GROUP_ID (0xE001)
#define J5_UART_GROUP_ID (0xE002)
#define J5_IMU_GROUP_ID (0xE003)

#define J5_IMAGE_GROUP_ID (0xE010)

typedef void (*recv_func_cb_t)(unsigned char * data, int data_len);

//connect the server using ip address
int connect_server(const char * server_ip, unsigned short  group_id, recv_func_cb_t recv_func_cb);

//disconnect the server
void disconnect_server(void);

//sent data to peer client
int transfer_data(const unsigned char * data, int data_len);

#endif
