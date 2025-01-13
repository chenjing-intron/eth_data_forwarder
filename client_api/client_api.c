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
#include "arm_control_packet.h"
#include "arm_control_api.h"
#include <semaphore.h>
#include <time.h>
#include <errno.h>

typedef struct arm_control_cmd
{
    uint32 cmd_index;
    arm_control_request_t req;
    arm_control_response_t resp;
    sem_t sem;
    struct arm_control_cmd *next_cmd;
} arm_control_cmd_t;

static uint32 cmd_index = 0;
static pthread_mutex_t cmd_list_mutex;
static arm_control_cmd_t *cmd_list = NULL;

static arm_control_cmd_t *create_arm_control_cmd(void)
{
    arm_control_cmd_t *new_cmd = (arm_control_cmd_t *)malloc(sizeof(arm_control_cmd_t));
    if (!new_cmd)
    {
        printf("new command fail\r\n");
        return NULL;
    }

    memset((void *)new_cmd, 0, sizeof(arm_control_cmd_t));

    pthread_mutex_lock(&cmd_list_mutex);

    cmd_index++;
    new_cmd->cmd_index = cmd_index;
    sem_init(&new_cmd->sem, 0, 0);

    if (!cmd_list)
    {
        cmd_list = new_cmd;
    }
    else
    {
        cmd_list->next_cmd = new_cmd;
    }

    pthread_mutex_unlock(&cmd_list_mutex);

    return new_cmd;
}

static void destroy_arm_control_cmd(arm_control_cmd_t *cmd)
{
    pthread_mutex_lock(&cmd_list_mutex);
    if (cmd_list == cmd)
    {
        cmd_list = cmd_list->next_cmd;
    }
    else
    {
        arm_control_cmd_t *prev = cmd_list;
        while (prev->next_cmd)
        {
            if (prev->next_cmd == cmd)
            {
                prev->next_cmd = cmd->next_cmd;
            }
            else
            {
                prev = prev->next_cmd;
            }
        }
    }
    pthread_mutex_unlock(&cmd_list_mutex);

    sem_destroy(&cmd->sem);
    free(cmd);
}

static void clean_arm_control_cmd(void)
{
    arm_control_cmd_t * cmd = NULL;

    pthread_mutex_lock(&cmd_list_mutex);
    while (cmd_list != NULL)
    {
        cmd = cmd_list;
        cmd_list = cmd_list->next_cmd;
        
        sem_destroy(&cmd->sem);
        free(cmd);
    }
    pthread_mutex_unlock(&cmd_list_mutex);
}

static void notify_response(const arm_control_response_t *resp)
{
    arm_control_cmd_t *p = NULL;

    pthread_mutex_lock(&cmd_list_mutex);

    p = cmd_list;
    while (p != NULL)
    {
        if (p->cmd_index == resp->cmd_index)
        {
            break;
        }
        else
        {
            p = p->next_cmd;
        }
    }

    if (p != NULL)
    {
        p->resp = *resp;
        sem_post(&p->sem);
    }

    pthread_mutex_unlock(&cmd_list_mutex);
}

#define DEG_CONVERSION_PARAM (4599.0f)

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
    uint8_t data[MAX_COMMAND_DATA_LEN + 6];
    field_data_t field[FIELD_NUM];
} tcp_client_t;

#define uint16_t unsigned short

static tcp_client_t client;
static struct sockaddr_in serv_addr;

static pthread_t tcp_receive_listen;

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
                        if (field_p->actual_len == CMD_RESP_DATA_LENGTH)
                        {
                            arm_control_response_t resp;
                            int j = 0;
                            memset((void *)&resp, 0, sizeof(resp));
                            memcpy((void *)&resp.command, field_p->data, CMD_RESP_DATA_LENGTH);

                            resp.command = ntohs(resp.command);
                            resp.command_result = ntohs(resp.command_result);
                            resp.cmd_index = ntohl(resp.cmd_index);
                            resp.action_index = ntohl(resp.action_index);
                            resp.gripper_state = ntohs(resp.gripper_state);

                            for (j = 0; j < MAX_JOINT_NUM; j++)
                            {
                                resp.joint_pos[j] = ntohl(resp.joint_pos[j]);
                            }

                            notify_response(&resp);
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

// connect the arm using ip address
int connect_arm(const char *server_ip)
{
    int sock = -1;

    init_client_rx_field(&client);
    pthread_mutex_init(&cmd_list_mutex, NULL);

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(7071);

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

    return 0;

err_out:
    close(sock);
    sock = -1;
    return -1;
}

void disconnect_arm(void)
{
    close(client.fd);
    client.fd = -1;
    pthread_cancel(tcp_receive_listen);
    pthread_join(tcp_receive_listen, NULL); 
    clean_arm_control_cmd();
    pthread_mutex_destroy(&cmd_list_mutex);
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

static int send_arm_cmd(const arm_control_request_t *req, arm_control_response_t *resp)
{
    arm_control_cmd_t *cmd = create_arm_control_cmd();
    ssize_t send_num = 0;
    struct timespec ts;
    int result = ERR_OK;

    if (!cmd)
    {
        return ERR_NEW_CMD_FAIL;
    }

    cmd->req = *req;
    cmd->req.cmd_index = htonl(cmd->cmd_index);
    cmd->req.checksum = htons(cal_checksum((uint8 *)&cmd->req.command, CMD_REQ_DATA_LENGTH));

    send_num = send(client.fd, &cmd->req, sizeof(arm_control_request_t), 0);
    if (send_num != sizeof(arm_control_request_t))
    {
        printf("error:send_num=%ld\r\n", send_num);

        if (send_num < 0)
        {
            printf("errno=%d,%s\r\n", errno, strerror(errno));
        }

        result = ERR_SEND_CMD_FAIL;
        goto RET_OUT;
    }

    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 5;

    if (sem_timedwait(&cmd->sem, &ts) == -1)
    {
        if (errno == ETIMEDOUT)
        {
            printf("sem_timedwait timed out\n");
            result = ERR_RECV_CMD_TIMEOUT;
        }
        else
        {
            perror("sem_timedwait failed");
            result = ERR_RECV_CMD_FAILED;
        }
        goto RET_OUT;
    }
    else
    {
        // printf("sem_timedwait succeeded\n");
        *resp = cmd->resp;
    }

RET_OUT:
    destroy_arm_control_cmd(cmd);
    return result;
}

// get the status of the ARM
int get_arm_status(int *arm_status)
{
    int ret = 0;
    arm_control_request_t req;
    arm_control_response_t resp;

    memset((void *)&req, 0, sizeof(req));
    req.magic = htons(MAGIC_WHOLE);
    req.command = htons(CMD_GET_ARM_STATUS);
    req.length = htons(CMD_REQ_DATA_LENGTH);

    ret = send_arm_cmd(&req, &resp);
    if (ret != ERR_OK)
    {
        return ret;
    }

    *arm_status = (int)resp.command_result;

    return ERR_OK;
}

// get the current position of the joints of the ARM
int get_joint_pos(joint_pos_t *j_pos)
{
    int ret = 0, i = 0;
    arm_control_request_t req;
    arm_control_response_t resp;

    memset((void *)&req, 0, sizeof(req));
    req.magic = htons(MAGIC_WHOLE);
    req.command = htons(CMD_GET_JOINT_POS);
    req.length = htons(CMD_REQ_DATA_LENGTH);

    ret = send_arm_cmd(&req, &resp);
    if (ret != ERR_OK)
    {
        return ret;
    }

    for (i = 0; i < MAX_JOINT_NUM; i++)
    {
        j_pos->pos[i] = (double)(((int)resp.joint_pos[i]) * 1.0f / DEG_CONVERSION_PARAM);
    }

    return ERR_OK;
}

// set the target position of the joints of the ARM
unsigned int set_joint_pos(const joint_pos_t *j_pos, unsigned int execute_time_us)
{
    int ret = 0, i = 0;
    arm_control_request_t req;
    arm_control_response_t resp;

    memset((void *)&req, 0, sizeof(req));
    req.magic = htons(MAGIC_WHOLE);
    req.command = htons(CMD_SET_JOINT_POS);
    req.length = htons(CMD_REQ_DATA_LENGTH);
    req.execute_time_us = htonl(execute_time_us);

    for (i = 0; i < MAX_JOINT_NUM; i++)
    {
        uint32 deg = (uint32)((int)(j_pos->pos[i] * DEG_CONVERSION_PARAM));
        req.joint_pos[i] = htonl(deg);
    }


    ret = send_arm_cmd(&req, &resp);
    if (ret != ERR_OK)
    {
        return ret;
    }

    return resp.action_index;
}

// get the status of the jonint action
int get_joint_action_status(unsigned int action_index)
{
    int ret = 0;
    arm_control_request_t req;
    arm_control_response_t resp;

    memset((void *)&req, 0, sizeof(req));
    req.magic = htons(MAGIC_WHOLE);
    req.command = htons(CMD_GET_JOINT_ACTION_STATUS);
    req.length = htons(CMD_REQ_DATA_LENGTH);
    req.action_index = htonl(action_index);

    ret = send_arm_cmd(&req, &resp);
    if (ret != ERR_OK)
    {
        return ret;
    }

    return resp.command_result;
}

// get the control state of the gripper
int get_gripper_state()
{
    int ret = 0;
    arm_control_request_t req;
    arm_control_response_t resp;

    memset((void *)&req, 0, sizeof(req));
    req.magic = htons(MAGIC_WHOLE);
    req.command = htons(CMD_GET_GRIPPER_STATE);
    req.length = htons(CMD_REQ_DATA_LENGTH);

    ret = send_arm_cmd(&req, &resp);
    if (ret != ERR_OK)
    {
        return ret;
    }

    return resp.gripper_state;
}

// set the control state of the gripper
int set_gripper_state(int gripper_state)
{
    int ret = 0;
    arm_control_request_t req;
    arm_control_response_t resp;

    memset((void *)&req, 0, sizeof(req));
    req.magic = htons(MAGIC_WHOLE);
    req.command = htons(CMD_SET_GRIPPER_STATE);
    req.length = htons(CMD_REQ_DATA_LENGTH);
    req.gripper_state = htons(gripper_state);

    ret = send_arm_cmd(&req, &resp);
    if (ret != ERR_OK)
    {
        return ret;
    }

    return resp.command_result;
}
