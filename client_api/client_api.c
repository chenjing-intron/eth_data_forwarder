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
#include <sys/uio.h>

static int filed_data_len[FIELD_NUM] =
    {
        FIELD_MAGIC_LEN,
        FIELD_LENGTH_LEN,
        MAX_COMMAND_DATA_LEN,
        FIELD_CHECKSUM_LEN,
};

#if 0
// 数据包结构
typedef struct {
    void *data;     // 数据指针
    size_t size;    // 数据长度
} DataPacket;

// 双向链表节点
typedef struct QueueNode {
    DataPacket packet;
    struct QueueNode *prev;
    struct QueueNode *next;
} QueueNode;

// 线程安全队列
typedef struct {
    QueueNode *head;            // 队列头节点
    QueueNode *tail;            // 队列尾节点
    pthread_mutex_t mutex;       // 互斥锁
    sem_t sem;                  // 信号量
    int count;
    int max_count;
    volatile int is_running;    // 队列运行标志
} ThreadSafeQueue;
#endif

typedef struct tcp_client
{
    int fd;
    int valid;
    int recv_state;
    recv_func_cb_t  recv_func_cb;
    unsigned short  group_id;
    uint8_t data[MAX_COMMAND_DATA_LEN + FIELD_MAGIC_LEN + FIELD_LENGTH_LEN + FIELD_CHECKSUM_LEN];
    field_data_t field[FIELD_NUM];

    //ThreadSafeQueue *queue;     // 发送队列
} tcp_client_t;

#define uint16_t unsigned short

static tcp_client_t client;
static struct sockaddr_in serv_addr;

static pthread_t tcp_receive_listen;

#if 0
static ThreadSafeQueue queue;
static pthread_t tcp_send_thread;

// 初始化队列
void queue_init(ThreadSafeQueue *queue) {
    queue->head = queue->tail = NULL;
    pthread_mutex_init(&queue->mutex, NULL);
    sem_init(&queue->sem, 0, 0);
    
    queue->count = 0;
    queue->max_count = 0;

    queue->is_running = 1;
}

// 销毁队列并释放资源
void queue_destroy(ThreadSafeQueue *queue) {
    pthread_mutex_lock(&queue->mutex);
    queue->is_running = 0;  // 停止队列运行

    // 释放所有未处理的数据包
    QueueNode *current = queue->head;
    while (current != NULL) {
        QueueNode *next = current->next;
        free(current->packet.data);
        free(current);
        current = next;
    }
    pthread_mutex_unlock(&queue->mutex);

    pthread_mutex_destroy(&queue->mutex);
    sem_destroy(&queue->sem);
}

// 向队列添加数据包（线程安全）
void enqueue(ThreadSafeQueue *queue, const void *data, size_t size) {
    // 创建新节点
    QueueNode *node = (QueueNode*)malloc(sizeof(QueueNode));
    node->packet.data = malloc(size);
    memcpy(node->packet.data, data, size);
    node->packet.size = size;
    node->prev = node->next = NULL;

    // 加锁操作队列
    pthread_mutex_lock(&queue->mutex);
    if (queue->tail == NULL) {
        queue->head = queue->tail = node;
    } else {
        node->prev = queue->tail;
        queue->tail->next = node;
        queue->tail = node;
    }

    queue->count++;
    if (queue->count > queue->max_count)
    {
        queue->max_count = queue->count;
        printf("max_count=%d\r\n", queue->max_count);
    }

    pthread_mutex_unlock(&queue->mutex);

    // 通知发送线程有新数据
    sem_post(&queue->sem);
}

// 从队列取出数据包（线程安全）
DataPacket dequeue(ThreadSafeQueue *queue) {
    sem_wait(&queue->sem);  // 等待信号量

    pthread_mutex_lock(&queue->mutex);
    if (!queue->is_running || queue->head == NULL) {
        pthread_mutex_unlock(&queue->mutex);
        return (DataPacket){NULL, 0};
    }

    // 取出头节点
    QueueNode *node = queue->head;
    if (queue->head == queue->tail) {
        queue->head = queue->tail = NULL;
    } else {
        queue->head = node->next;
        queue->head->prev = NULL;
    }

    queue->count--;
    printf("dequeue:count=%d\r\n", queue->count);

    pthread_mutex_unlock(&queue->mutex);

    DataPacket packet = node->packet;
    free(node);  // 释放节点内存（数据内存由发送线程释放）
    return packet;
}
#endif    

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
        uint32 recv_len = *((uint32 *)field_p->data);
        recv_len = ntohl(recv_len);

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
        int check_data_len = MAX_CHECK_DATA_LEN + MSG_FIELD_COMMAND_LENGTH;
        uint16_t cal_checksum = 0;
        uint16_t recv_checksum = *((uint16_t *)field_p->data);
        recv_checksum = ntohs(recv_checksum);

        if (client_p->field[FIELD_DATA].actual_len < check_data_len)
        {
            check_data_len = client_p->field[FIELD_DATA].actual_len;
        }

        for (i = 0; i < check_data_len; i++)
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

static int sent_group_register_request(void);

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
        printf("\nConnection server Failed \n");
        goto err_out;
    }

    client.fd = sock;

    int ret = sent_group_register_request();
    if (ret != ERR_OK)
    {
        printf("sent group_register_request fialed,ret=%d\r\n", ret);
    }

    return;

err_out:
    close(sock);
    sock = -1;

    return;
}

#if 0
static void *client_send_func(void *ptr)
{
    tcp_client_t *client_p = (tcp_client_t *)ptr;
    ThreadSafeQueue *queue = client_p->queue;

    while (queue->is_running) {
        DataPacket packet = dequeue(queue);
        if (packet.data == NULL) continue;

        // 完整发送数据（处理部分发送的情况）
        size_t total_sent = 0;
        while (total_sent < packet.size) {
            ssize_t sent = send(client_p->fd, 
                              (char*)packet.data + total_sent, 
                              packet.size - total_sent, 
                              0);
            if (sent <= 0) {
                perror("send failed");
                break;
            }
            total_sent += sent;
        }

        free(packet.data);  // 释放数据内存
    }
    return NULL;
}
#endif    

static void *client_receive_func(void *ptr)
{
    tcp_client_t *client_p = (tcp_client_t *)ptr;
    fd_set readfds;
    struct timeval timeout;
    int maxfd = -1;
    #define MAX_BUF_LEN (1*1024*1024)
    static uint8_t buf[MAX_BUF_LEN] = {0};

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
            printf("---reconnect server---\r\n");
            continue;
        }

        timeout.tv_sec = 2; // 设置超时时间
        timeout.tv_usec = 0;

        int ret = select(maxfd + 1, &readfds, NULL, NULL, &timeout);
        if (ret == -1)
        {
            // 错误处理
            printf("select ret < 0\r\n");
        }
        else if (ret == 0)
        {
            // 超时处理
            //printf("select ret == 0\r\n");
        }
        else
        {
            if (FD_ISSET(client_p->fd, &readfds))
            {
                int recv_number = 0;
                while (1)
                {
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
                        if (bytes < 0)
                        {
                            if (recv_number == 0)
                            {
                                // client error
                                printf("lib(bytes < 0):client error \r\n");
                                //close(client_p->fd);
                                //client_p->fd = -1;
                            }
                            
                            break;
                        }
                        else
                        {
                            if (recv_number == 0)
                            {
                                // client error
                                printf("lib(recv_number == 0):client error \r\n");
                                close(client_p->fd);
                                client_p->fd = -1;
                                break;
                            }
                            #if 1
                            else
                            {
                                break;
                            }
                            #endif
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
int connect_server(const char *server_ip, unsigned short  group_id, recv_func_cb_t recv_func_cb)
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

    int send_buf_size = 9 * 1024 * 1024;
    setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &send_buf_size, sizeof(send_buf_size));

    int recv_buf_size = 12 * 1024 * 1024;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &recv_buf_size, sizeof(recv_buf_size));

    #if 0
    queue_init(&queue);
    client.queue = &queue;
    #endif

    client.fd = sock;
    client.recv_func_cb = recv_func_cb;
    client.group_id = group_id;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE); 

    if (pthread_create(&tcp_receive_listen, &attr, client_receive_func, &client) != 0)
    {
        perror("pthread_create");
        pthread_attr_destroy(&attr); 
        goto err_out;
    }

    #if 0
    if (pthread_create(&tcp_send_thread, &attr, client_send_func, &client) != 0)
    {
        perror("pthread_create");
        pthread_attr_destroy(&attr); 
        goto err_out;
    }
    #endif

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

#ifdef PRINT_DATA
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
#endif

static int send_client_data(const unsigned char * data, int data_len)
{
    ssize_t send_num = 0;
    int result = ERR_OK;
    struct timeval tv;
	gettimeofday(&tv, NULL);

#ifdef PRINT_DATA
    print_data(data, data_len);
#endif
    if (client.fd < 0)
    {
        return ERR_SEND_CMD_FAIL;
    }

    //printf("---111 send data_len%d, time=%ld.%ld.%ld \r\n", data_len, tv.tv_sec, tv.tv_usec / 1000, tv.tv_usec % 1000);

    send_num = send(client.fd, data, data_len, 0);
    if (send_num != data_len)
    {
        printf("client_api error:send_num=%ld\r\n", send_num);

        if (send_num < 0)
        {
            printf("client errno=%d,%s\r\n", errno, strerror(errno));
        }

        result = ERR_SEND_CMD_FAIL;
    }

    //gettimeofday(&tv, NULL);
    //printf("---222 send data_len%d, time=%ld.%ld.%ld \r\n", data_len, tv.tv_sec, tv.tv_usec / 1000, tv.tv_usec % 1000);

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
    req.group_id = htons(client.group_id);
    req.length = htonl(CLIENT_REGISTER_DATA_LENGTH);
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
    //int ret = ERR_OK;
    unsigned char * p = NULL;
    int check_data_len = MAX_CHECK_DATA_LEN;
    

    struct timeval tv;
	gettimeofday(&tv, NULL);
    //printf("   ### 111 send data_len%d, time=%ld.%ld.%ld \r\n", data_len, tv.tv_sec, tv.tv_usec / 1000, tv.tv_usec % 1000);

    unsigned char header[FIELD_MAGIC_LEN + FIELD_LENGTH_LEN + MSG_FIELD_COMMAND_LENGTH] = {0x00};  // 包头模板
    unsigned char footer[FIELD_CHECKSUM_LEN] = {0x00, 0x00};  // 包尾模板

    if (client.fd < 0)
    {
        return ERR_SEND_CMD_FAIL;
    }

    struct iovec iov[3];
    iov[0].iov_base = header;
    iov[0].iov_len = sizeof(header);
    iov[1].iov_base = (void*)data;
    iov[1].iov_len = data_len;
    iov[2].iov_base = footer;
    iov[2].iov_len = sizeof(footer);

    p = header;

    //memset((void*)client_data_ptr, 0, MAX_COMMAND_DATA_LEN + FIELD_MAGIC_LEN + FIELD_LENGTH_LEN + FIELD_CHECKSUM_LEN);
    *((uint16 *)p) = htons(MAGIC_WHOLE);
    p += sizeof(uint16);

    *((uint32 *)p) = htonl(data_len + MSG_FIELD_COMMAND_LENGTH);
    p += sizeof(uint32);

    *((uint16 *)p) = htons(CLIENT_CMD_TRANSFER_DATA);
    p += sizeof(uint16);

    p = footer;

    if (data_len < check_data_len)
    {
        check_data_len = data_len;
    }

    *((uint16 *)p) = htons(cal_checksum((uint8 *)data, check_data_len) + cal_checksum((uint8 *)&header[FIELD_MAGIC_LEN + FIELD_LENGTH_LEN], MSG_FIELD_COMMAND_LENGTH));

    //gettimeofday(&tv, NULL);
    //printf("   ### 222 send data_len%d, time=%ld.%ld.%ld \r\n", data_len, tv.tv_sec, tv.tv_usec / 1000, tv.tv_usec % 1000);

    ssize_t bytes_sent = writev(client.fd, iov, 3);
    if (bytes_sent < 0) {
        perror("writev failed");
    }

    //gettimeofday(&tv, NULL);
    //printf("   ###  333 send data_len%d, time=%ld.%ld.%ld \r\n", data_len, tv.tv_sec, tv.tv_usec / 1000, tv.tv_usec % 1000);


    #if 0
    //unsigned char * client_data_ptr = NULL;
    static unsigned char  client_data_ptr[MAX_COMMAND_DATA_LEN + FIELD_MAGIC_LEN + FIELD_LENGTH_LEN + FIELD_CHECKSUM_LEN];
    
    if (data_len > (int)(MAX_COMMAND_DATA_LEN - MSG_FIELD_COMMAND_LENGTH))
    {
        return -1;
    }

    #if 0
    client_data_ptr = (unsigned char *)malloc(MAX_COMMAND_DATA_LEN + FIELD_MAGIC_LEN + FIELD_LENGTH_LEN + FIELD_CHECKSUM_LEN);
    if (client_data_ptr == NULL)
    {
        printf("malloc failed\r\n");
        return -1;
    }
    #endif

    p = client_data_ptr;

    //memset((void*)client_data_ptr, 0, MAX_COMMAND_DATA_LEN + FIELD_MAGIC_LEN + FIELD_LENGTH_LEN + FIELD_CHECKSUM_LEN);
    *((uint16 *)p) = htons(MAGIC_WHOLE);
    p += sizeof(uint16);

    *((uint32 *)p) = htonl(data_len + MSG_FIELD_COMMAND_LENGTH);
    p += sizeof(uint32);

    *((uint16 *)p) = htons(CLIENT_CMD_TRANSFER_DATA);
    p += sizeof(uint16);

    gettimeofday(&tv, NULL);
    printf("   ### aaa send data_len%d, time=%ld.%ld.%ld \r\n", data_len, tv.tv_sec, tv.tv_usec / 1000, tv.tv_usec % 1000);

    memcpy((void*)p, data, data_len);
    p += data_len;

    *((uint16 *)p) = htons(cal_checksum((uint8 *)&client_data_ptr[sizeof(uint16) + sizeof(uint32)], data_len + MSG_FIELD_COMMAND_LENGTH));

    //enqueue(client.queue, client_data_ptr, data_len + MSG_FIELD_COMMAND_LENGTH + FIELD_MAGIC_LEN + FIELD_LENGTH_LEN + FIELD_CHECKSUM_LEN);

    gettimeofday(&tv, NULL);
    printf("   ### 222 send data_len%d, time=%ld.%ld.%ld \r\n", data_len, tv.tv_sec, tv.tv_usec / 1000, tv.tv_usec % 1000);

    ret = send_client_data((const unsigned char *)client_data_ptr, data_len + MSG_FIELD_COMMAND_LENGTH + FIELD_MAGIC_LEN + FIELD_LENGTH_LEN + FIELD_CHECKSUM_LEN);

    gettimeofday(&tv, NULL);
    printf("   ### 333 send data_len%d, time=%ld.%ld.%ld \r\n", data_len, tv.tv_sec, tv.tv_usec / 1000, tv.tv_usec % 1000);

    //free(client_data_ptr);
    
    #if 1
    if (ret != ERR_OK)
    {
        return ret;
    }
    #endif
    #endif

    return ERR_OK;
}
