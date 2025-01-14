#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>
#include <sched.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include <errno.h>
#include "client_api_packet.h"
#include "client_api.h"

static int filed_data_len[FIELD_NUM] =
    {
        FIELD_MAGIC_LEN,
        FIELD_LENGTH_LEN,
        MAX_COMMAND_DATA_LEN,
        FIELD_CHECKSUM_LEN,
};

typedef struct tcp_client
{
    int fd;
    int valid;
    int recv_state;
    recv_func_cb_t  recv_func_cb;
    uint8_t data[MAX_COMMAND_DATA_LEN + 6];
    field_data_t field[FIELD_NUM];
} tcp_client_t;

#define uint16_t unsigned short

static tcp_client_t client;
static struct sockaddr_in serv_addr;

static pthread_t tcp_receive_listen;

static void notify_response(uint8 * data, int data_len)
{
    if (client.recv_func_cb != NULL)
    {
        client.recv_func_cb(data, data_len);
    }
}

static void init_client_rx_field(tcp_client_t *client_p)
{
    int i = 0, offset = 0;
    for (i = 0; i < RECV_STATE_MAX; i++)
    {
        client_p->field[i].data = client_p->data + offset;
        offset += filed_data_len[i];

        client_p->field[i].cur_len = 0;
        client_p->field[i].actual_len = filed_data_len[i];
        client_p->field[i].max_len = filed_data_len[i];
    }
}

static void reset_client_rx_state(tcp_client_t *client_p)
{
    int i = 0;
    for (i = 0; i < RECV_STATE_MAX; i++)
    {
        client_p->field[i].cur_len = 0;
    }

    client_p->recv_state = RECV_MAGIC;
}

static int check_field_data(tcp_client_t *client_p, int recv_state)
{
    field_data_t *field_p = &client_p->field[recv_state];

    // check the field value
    switch (recv_state)
    {
    case RECV_MAGIC:
    {
        uint16 magic_value = ntohs(*((uint16 *)field_p->data));
        if (magic_value != MAGIC_WHOLE)
        {
            printf("wrong magic value:0x%x,0x%x\r\n", field_p->data[0], field_p->data[1]);
            reset_client_rx_state(client_p);
            return 1;
        }
        break;
    }

    case RECV_LEN:
    {
        uint16_t recv_len = *((uint16_t *)field_p->data);
        recv_len = ntohs(recv_len);

        // printf("recv_len=%d\r\n", recv_len);

        if (recv_len > MAX_COMMAND_DATA_LEN || recv_len == 0)
        {
            reset_client_rx_state(client_p);
            return 1;
        }
        else
        {
            client_p->field[FIELD_DATA].actual_len = recv_len;
        }
        break;
    }

    case RECV_CHECKSUM:
    {
        int i = 0;
        uint16_t cal_checksum = 0;
        uint16_t recv_checksum = *((uint16_t *)field_p->data);
        recv_checksum = ntohs(recv_checksum);
        for (i = 0; i < client_p->field[FIELD_DATA].actual_len; i++)
        {
            cal_checksum += client_p->field[FIELD_DATA].data[i];
        }

        if (recv_checksum != cal_checksum)
        {
            printf("wrong checksum:recv_checksum=%d,cal_checksum=%d\r\n", recv_checksum, cal_checksum);
            reset_client_rx_state(client_p);
            return 1;
        }

        break;
    }

    default:
    {
        break;
    }
    }

    return 0;
}

static void process_client_data(tcp_client_t *client_p, uint8_t *data, int data_len)
{
    int i = 0;
    for (i = 0; i < data_len; i++)
    {
        if (client_p->recv_state < RECV_STATE_MAX)
        {
            field_data_t *field_p = &client_p->field[client_p->recv_state];
            field_p->data[field_p->cur_len] = data[i];
            field_p->cur_len++;
            if (field_p->cur_len == field_p->actual_len)
            {
                int check_result = check_field_data(client_p, client_p->recv_state);

                if (0 == check_result)
                {
                    client_p->recv_state++;
                    if (client_p->recv_state >= RECV_STATE_MAX)
                    {
                        // got a command req or resp
                        field_p = &client_p->field[RECV_DATA];
                        if (field_p->actual_len > (int)MSG_FIELD_COMMAND_LENGTH)
                        {
                            int data_len = field_p->actual_len - MSG_FIELD_COMMAND_LENGTH;
                            
                            uint16 command = ntohs(*((uint16*)field_p->data));

                            if (CLIENT_CMD_REGISTER_GROUP == command)
                            {
                                
                            } 
                            else if (CLIENT_CMD_TRANSFER_DATA == command)
                            {
                                notify_response(field_p->data + sizeof(uint16), data_len);
                            }
                            else
                            {
                                printf("error:wrong client cmd, cmd=0x%x\r\n", command);
                            }                            
                        }
                        else
                        {
                            printf("error:wrong response data len\r\n");
                        }

                        reset_client_rx_state(client_p);
                    }
                }
            }
        }
        else
        {
            printf("this is an exception in rx\r\n");
            reset_client_rx_state(client_p);
        }
    }
}

static void client_reconnect_server(void)
{
    int sock = -1;
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        return;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        // printf("\nConnection Failed \n");
        goto err_out;
    }

    client.fd = sock;

    return;

err_out:
    close(sock);
    sock = -1;

    return;
}

static void *client_receive_func(void *ptr)
{
    tcp_client_t *client_p = (tcp_client_t *)ptr;
    fd_set readfds;
    struct timeval timeout;
    int maxfd = -1;

    if (!client_p)
    {
        printf("invalid thread param\r\n");
        return NULL;
    }

    while (1)
    {
        FD_ZERO(&readfds);
        maxfd = -1;

        FD_SET(client_p->fd, &readfds);
        maxfd = client_p->fd;

        if (maxfd < 0)
        {
            client_reconnect_server();
            usleep(2000000);
            continue;
        }

        timeout.tv_sec = 2; // 设置超时时间
        timeout.tv_usec = 0;

        int ret = select(maxfd + 1, &readfds, NULL, NULL, &timeout);
        if (ret == -1)
        {
            // 错误处理
        }
        else if (ret == 0)
        {
            // 超时处理
        }
        else
        {
            if (FD_ISSET(client_p->fd, &readfds))
            {
                int recv_number = 0;
                while (1)
                {
                    uint8_t buf[1024] = {0};

                    // printf("before recv \r\n");
                    int bytes = recv(client_p->fd, buf, sizeof(buf), MSG_DONTWAIT);
                    // printf("recv:bytes=%d\r\n", bytes);

                    if (bytes > 0)
                    {
                        process_client_data(client_p, buf, bytes);
                        recv_number += bytes;
                    }
                    else
                    {
                        if (recv_number == 0)
                        {
                            // client error
                            close(client_p->fd);
                            client_p->fd = -1;
                            break;
                        }
                        else
                        {
                            break;
                        }
                    }
                }
            }
        }
    }

    return NULL;
}

static int sent_group_register_request(void);

// connect the arm using ip address
int connect_server(const char *server_ip, recv_func_cb_t recv_func_cb)
{
    int sock = -1, ret = ERR_OK;

    init_client_rx_field(&client);

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(SERVER_PORT_NUM);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if (inet_pton(AF_INET, server_ip, &serv_addr.sin_addr) <= 0)
    {
        printf("\nInvalid address/ Address not supported \n");
        goto err_out;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("\nConnection Failed \n");
        goto err_out;
    }

    client.fd = sock;
    client.recv_func_cb = recv_func_cb;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE); 

    if (pthread_create(&tcp_receive_listen, &attr, client_receive_func, &client) != 0)
    {
        perror("pthread_create");
        pthread_attr_destroy(&attr); 
        goto err_out;
    }

    pthread_attr_destroy(&attr); 

    ret = sent_group_register_request();
    if (ret != ERR_OK)
    {
        printf("sent group_register_request fialed,ret=%d\r\n", ret);
    }

    return 0;

err_out:
    close(sock);
    sock = -1;
    return -1;
}

void disconnect_server(void)
{
    close(client.fd);
    client.fd = -1;
    pthread_cancel(tcp_receive_listen);
    pthread_join(tcp_receive_listen, NULL); 
}

static uint16 cal_checksum(uint8 *data, int data_len)
{
    int i = 0;
    uint16_t cal_checksum = 0;
    for (i = 0; i < data_len; i++)
    {
        cal_checksum += data[i];
    }

    return cal_checksum;
}

static void print_data(const unsigned char *data, int data_len)
{
   int i = 0;

   printf("---the data is:\r\n");
   for (i = 0; i < data_len; i++)
   {
      printf("0x%x ", data[i]);
      if (0 == ((i + 1) % 16))
      {
         printf("\r\n");
      }
   }
   printf("\r\n");
}

static int send_client_data(const unsigned char * data, int data_len)
{
    ssize_t send_num = 0;
    int result = ERR_OK;

    print_data(data, data_len);

    send_num = send(client.fd, data, data_len, 0);
    if (send_num != data_len)
    {
        printf("error:send_num=%ld\r\n", send_num);

        if (send_num < 0)
        {
            printf("errno=%d,%s\r\n", errno, strerror(errno));
        }

        result = ERR_SEND_CMD_FAIL;
    }

    return result;
}

// get the status of the ARM
static int sent_group_register_request(void)
{
    int ret = 0;
    client_register_request_t req;

    memset((void *)&req, 0, sizeof(req));
    req.magic = htons(MAGIC_WHOLE);
    req.command = htons(CLIENT_CMD_REGISTER_GROUP);
    req.group_id = htons(AGV_CAN_GROUP_ID);
    req.length = htons(CLIENT_REGISTER_DATA_LENGTH);
    req.checksum = htons(cal_checksum((uint8 *)&req.command, CLIENT_REGISTER_DATA_LENGTH));

    ret = send_client_data((const unsigned char *)&req, sizeof(client_register_request_t));
    if (ret != ERR_OK)
    {
        return ret;
    }

    return ERR_OK;
}

int transfer_data(const unsigned char * data, int data_len)
{
    int ret = ERR_OK;

    unsigned char client_data[MAX_COMMAND_DATA_LEN + FIELD_MAGIC_LEN + FIELD_LENGTH_LEN + FIELD_CHECKSUM_LEN];
    unsigned char * p = client_data;
    if (data_len > (int)(MAX_COMMAND_DATA_LEN - MSG_FIELD_COMMAND_LENGTH))
    {
        return -1;
    }

    memset((void*)client_data, 0, sizeof(client_data));
    *((uint16 *)p) = htons(MAGIC_WHOLE);
    p += sizeof(uint16);

    *((uint16 *)p) = htons(data_len + MSG_FIELD_COMMAND_LENGTH);
    p += sizeof(uint16);

    *((uint16 *)p) = htons(CLIENT_CMD_TRANSFER_DATA);
    p += sizeof(uint16);

    memcpy((void*)p, data, data_len);
    p += data_len;

    *((uint16 *)p) = htons(cal_checksum((uint8 *)&client_data[sizeof(uint16) + sizeof(uint16)], data_len + MSG_FIELD_COMMAND_LENGTH));

    ret = send_client_data((const unsigned char *)client_data, data_len + MSG_FIELD_COMMAND_LENGTH + FIELD_MAGIC_LEN + FIELD_LENGTH_LEN + FIELD_CHECKSUM_LEN);
    if (ret != ERR_OK)
    {
        return ret;
    }

    return ERR_OK;
}
