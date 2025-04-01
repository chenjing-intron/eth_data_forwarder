/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>
#include <sched.h>
#include <arpa/inet.h>
#include "client_api_packet.h"
#include <errno.h>
#include "server.h"
#include <sys/resource.h>

#define MAX_TCP_CLIENT_NUM (10)

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
   uint16_t group_id;
   uint8_t data[MAX_COMMAND_DATA_LEN + FIELD_MAGIC_LEN + FIELD_LENGTH_LEN + FIELD_CHECKSUM_LEN];
   field_data_t field[FIELD_NUM];
   struct tcp_client *peer_client;
} tcp_client_t;

typedef struct tcp_server
{
   int listen_fd;
   pthread_mutex_t mutex;
   tcp_client_t clients[MAX_TCP_CLIENT_NUM];
} tcp_server_t;

static tcp_server_t server;

static OSAL_THREAD_HANDLE tcp_thread_listen;
static OSAL_THREAD_HANDLE tcp_receive_listen;

void enable_core_dumps() {
    struct rlimit core_limit;

    // 获取当前限制
    getrlimit(RLIMIT_CORE, &core_limit);

    // 设置为不限制
    core_limit.rlim_cur = core_limit.rlim_max;
    setrlimit(RLIMIT_CORE, &core_limit);

    system("mkdir -p /userdata/coredump");
    system("echo \"/userdata/coredump/core.%e.%p.%t\" > /proc/sys/kernel/core_pattern");

    printf("Core dump size limit set to %ld\n", core_limit.rlim_cur);
}

static void init_client_rx_field(tcp_client_t *client_p)
{
   int i = 0, offset = 0;
   for (i = 0; i < FIELD_NUM; i++)
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
   for (i = 0; i < FIELD_NUM; i++)
   {
      client_p->field[i].cur_len = 0;
   }

   client_p->recv_state = RECV_MAGIC;
}

static int check_field_data(tcp_client_t *client_p, int recv_state)
{
   field_data_t *field_p = &client_p->field[recv_state];

   //printf("check_field_data: recv_state=%d\r\n", recv_state);

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
      uint16_t recv_len = *((uint16 *)field_p->data);
      recv_len = ntohs(recv_len);

      // printf("recv_len=%d\r\n", recv_len);

      if (recv_len > MAX_COMMAND_DATA_LEN || recv_len == 0)
      {
         printf("wrong recv_len=%d\r\n", recv_len);

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
      uint16_t recv_checksum = *((uint16 *)field_p->data);
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
      else
      {
         //printf("ok checksum:recv_checksum=%d,cal_checksum=%d\r\n", recv_checksum, cal_checksum);
      }

      break;
   }

   default:
   {
      break;
   }
   }

   //printf("check_field_data: recv_state=%d end\r\n", recv_state);

   return 0;
}

#if 0
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
#endif

#if 0
static void print_data(uint8 *data, int data_len)
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

static void notify_response(tcp_client_t *client_p, uint8 *data, int data_len)
{
   ssize_t send_num = 0;

   send_num = send(client_p->fd, data, data_len, 0);
   if (send_num != data_len)
   {
      printf("error:send_num=%ld\r\n", send_num);

      if (send_num < 0)
      {
         printf("errno=%d,%s\r\n", errno, strerror(errno));
      }
   }
}

static void process_client_data(tcp_client_t *client_p, uint8_t *data, int data_len)
{
   int i = 0;

   //print_data(data, data_len);

   if (client_p->group_id != 0 && client_p->peer_client != NULL)
   {
      notify_response(client_p->peer_client, data, data_len);
      return;
   }

   for (i = 0; i < data_len; i++)
   {
      if (client_p->recv_state < RECV_STATE_MAX)
      {
         field_data_t *field_p = &client_p->field[client_p->recv_state];

          //printf("field=%d cur_len=%d actual_len=%d\r\n", client_p->recv_state, field_p->cur_len, field_p->actual_len);

         field_p->data[field_p->cur_len] = data[i];

         field_p->cur_len++;

         if (field_p->cur_len >= field_p->actual_len)
         {
            int check_result = check_field_data(client_p, client_p->recv_state);

            if (0 == check_result)
            {
               client_p->recv_state++;
               if (client_p->recv_state >= RECV_STATE_MAX)
               {
                  // get a command req or resp
                  // printf("get ok packet\r\n");

                  field_p = &client_p->field[RECV_DATA];
                  if (field_p->actual_len > (int)MSG_FIELD_COMMAND_LENGTH)
                  {
                     int data_len = field_p->actual_len - MSG_FIELD_COMMAND_LENGTH;

                     uint16 command = ntohs(*((uint16 *)field_p->data));

                     if (CLIENT_CMD_REGISTER_GROUP == command)
                     {
                        if (data_len >= (int)MSG_FIELD_GROUP_ID_LENGTH)
                        {
                           int j = 0;
                           uint16 group_id = ntohs(*((uint16 *)(field_p->data + MSG_FIELD_COMMAND_LENGTH)));
                           printf("recv client register:group_id=0x%x\r\n", group_id);

                           client_p->group_id = group_id;

                           for (j = 0; j < MAX_TCP_CLIENT_NUM; j++)
                           {
                              if (client_p != &server.clients[j] && server.clients[j].valid && server.clients[j].fd >= 0 && server.clients[j].group_id == group_id && group_id != 0)
                              {
                                 client_p->peer_client = &server.clients[j];
                                 server.clients[j].peer_client = client_p;
                                 printf("matched client ofr group_id=0x%x\r\n", group_id);
                                 break;
                              }
                           }
                        }
                        else
                        {
                           printf("error:wrong client register cmd, data_len=%d\r\n", data_len);
                        }
                     }
                     else if (CLIENT_CMD_TRANSFER_DATA == command)
                     {
                     }
                     else
                     {
                        printf("error:wrong client cmd, cmd=0x%x\r\n", command);
                     }
                  }
                  else
                  {
                     printf("error:wrong request data len\r\n");
                  }

                  reset_client_rx_state(client_p);
               }
            }
            else
            {
               printf("check_field_data: result=%d\r\n", check_result);
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

static OSAL_THREAD_FUNC server_listen_func(void *ptr)
{
   tcp_server_t *server_p = (tcp_server_t *)ptr;

   if (!server_p)
   {
      printf("invalid thread param\r\n");
      return;
   }

   while (1)
   {
      int new_socket = -1, i = 0;
      struct sockaddr_in address;
      int addrlen = sizeof(address);

      if ((new_socket = accept(server_p->listen_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen)) < 0)
      {
         perror("accept");
         continue;
      }

      // printf("Connection accepted\n");

      pthread_mutex_lock(&server.mutex);
      for (i = 0; i < MAX_TCP_CLIENT_NUM; i++)
      {
         if (!server_p->clients[i].valid)
         {
            server_p->clients[i].fd = new_socket;

            init_client_rx_field(&server_p->clients[i]);

            server_p->clients[i].group_id = 0;
            server_p->clients[i].peer_client = NULL;
            server_p->clients[i].recv_state = RECV_MAGIC;

            server_p->clients[i].valid = 1;

            // pthread_mutex_unlock(&server.mutex);

            break;
         }
      }
      pthread_mutex_unlock(&server.mutex);

      if (i == MAX_TCP_CLIENT_NUM)
      {
         printf("error:the client connection beyond the limits of %d\r\n", MAX_TCP_CLIENT_NUM);
      }
   }
}

static OSAL_THREAD_FUNC server_receive_func(void *ptr)
{
   tcp_server_t *server_p = (tcp_server_t *)ptr;
   fd_set readfds;
   struct timeval timeout;
   int maxfd = -1, i = 0;

   if (!server_p)
   {
      printf("invalid thread param\r\n");
      return;
   }

   while (1)
   {
      FD_ZERO(&readfds);
      maxfd = -1;

      pthread_mutex_lock(&server.mutex);
      for (i = 0; i < MAX_TCP_CLIENT_NUM; i++)
      {
         if (server_p->clients[i].valid && server_p->clients[i].fd >= 0)
         {
            FD_SET(server_p->clients[i].fd, &readfds);
            if (maxfd < server_p->clients[i].fd)
            {
               maxfd = server_p->clients[i].fd;
            }
         }
      }
      pthread_mutex_unlock(&server.mutex);

      if (maxfd < 0)
      {
         usleep(1000000);
         continue;
      }

      timeout.tv_sec = 2; // 设置超时时间
      timeout.tv_usec = 0;

      int ret = select(maxfd + 1, &readfds, NULL, NULL, &timeout);
      // printf("select:ret=%d\r\n", ret);

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
         pthread_mutex_lock(&server.mutex);

         for (i = 0; i < MAX_TCP_CLIENT_NUM; i++)
         {
            if (server_p->clients[i].valid && server_p->clients[i].fd >= 0)
            {
               // printf("one socket:fd=%d \r\n", server_p->clients[i].fd);
               if (FD_ISSET(server_p->clients[i].fd, &readfds))
               {
                  int recv_number = 0;
                  while (1)
                  {
                     uint8_t buf[1024] = {0};
                     // printf("before recv \r\n");
                     int bytes = recv(server_p->clients[i].fd, buf, sizeof(buf), MSG_DONTWAIT);
                     // printf("recv:bytes=%d\r\n", bytes);

                     if (bytes > 0)
                     {
                        process_client_data(&server_p->clients[i], buf, bytes);
                        recv_number += bytes;
                     }
                     else
                     {
                        if (recv_number == 0)
                        {
                           // client error
                           if (server_p->clients[i].peer_client != NULL)
                           {
                              server_p->clients[i].peer_client->peer_client = NULL;
                              server_p->clients[i].peer_client = NULL;
                           }

                           //printf("client err : disconnect\r\n");

                           server_p->clients[i].valid = 0;
                           close(server_p->clients[i].fd);
                           server_p->clients[i].fd = -1;
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

         pthread_mutex_unlock(&server.mutex);
      }
   }
}

static int osal_thread_create(void *thandle, int stacksize, void *func, void *param)
{
   int ret;
   pthread_attr_t attr;
   pthread_t *threadp;

   threadp = thandle;
   pthread_attr_init(&attr);
   pthread_attr_setstacksize(&attr, stacksize);
   ret = pthread_create(threadp, &attr, func, param);
   if (ret < 0)
   {
      return 0;
   }
   return 1;
}

int create_tcp_server(uint16_t port)
{
   int server_fd = -1;
   int opt = 1;
   struct sockaddr_in address;

   // init server data
   server.listen_fd = -1;
   pthread_mutex_init(&server.mutex, NULL);

   // Creating socket file descriptor
   if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
   {
      perror("socket failed");
      return -1;
   }

   // Forcefully attaching socket to the port 8080
   if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
   {
      perror("setsockopt");
      goto err_setsockopt;
   }

   address.sin_family = AF_INET;
   address.sin_addr.s_addr = INADDR_ANY;
   address.sin_port = htons(port);

   // Forcefully attaching socket to the port 8080
   if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
   {
      perror("bind failed");
      goto err_setsockopt;
   }
   if (listen(server_fd, 5) < 0)
   {
      perror("listen");
      goto err_setsockopt;
   }

   server.listen_fd = server_fd;

   osal_thread_create(&tcp_thread_listen, 128000, &server_listen_func, &server);
   osal_thread_create(&tcp_receive_listen, 128000, &server_receive_func, &server);

   return 0;

err_setsockopt:
   close(server_fd);
   server_fd = -1;
   return -1;
}

int main(int argc, char *argv[])
{
   int ret = 0;

   (void)argc;
   (void)argv;

   enable_core_dumps();

   #if 1
   // Set the scheduling policy to SCHED_FIFO
   struct sched_param param;
   param.sched_priority = 99; // Highest real-time priority

   // Set the scheduling policy and priority
   if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
   {
      system("echo 950000 > /sys/fs/cgroup/cpu/user.slice/cpu.rt_runtime_us");

      if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
      {
      	perror("sched_setscheduler");
      	exit(EXIT_FAILURE);
      }
   }

   // Print the new scheduling policy and priority
   int policy = sched_getscheduler(0);
   int priority = sched_getparam(0, &param);

   printf("New scheduling policy: %d\n", policy);
   printf("New priority: %d,%d\n", param.sched_priority, priority);
#endif 

   printf("ethernet data forwarder\n");

   ret = create_tcp_server(SERVER_PORT_NUM);
   printf("create_tcp_server ret=%d\r\n", ret);

   while (1)
   {
      usleep(10000000);
   }

   printf("End program\n");
   return (0);
}
