#ifndef __CONTROL_API_H__
#define __CONTROL_API_H__

#define ERR_OK               (0)
#define ERR_NEW_CMD_FAIL     (-1)
#define ERR_SEND_CMD_FAIL    (-2)
#define ERR_RECV_CMD_TIMEOUT (-3)
#define ERR_RECV_CMD_FAILED  (-4)

typedef void (*recv_func_cb_t)(unsigned char * data, int data_len);

//connect the server using ip address
int connect_server(const char * server_ip, recv_func_cb_t recv_func_cb);

//disconnect the server
void disconnect_server(void);


#endif
