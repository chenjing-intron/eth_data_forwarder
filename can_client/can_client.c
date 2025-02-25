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
#include <pthread.h>
#include "client_api.h"
#include <time.h>
#include <errno.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#define CAN_NAME "can0"
static int can_fd = -1;

pthread_mutex_t can_mutex = PTHREAD_MUTEX_INITIALIZER;

#include <pthread.h>

#define OSAL_THREAD_HANDLE pthread_t *
#define OSAL_THREAD_FUNC void
#define OSAL_THREAD_FUNC_RT void

static OSAL_THREAD_HANDLE can_thread_recv;

#ifdef PRINT_DATA
static void print_data(unsigned char *data, int data_len)
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

void client_recv_func(unsigned char * data, int data_len)
{
   int ret = 0;

   #ifdef PRINT_DATA
   //print_data(data, data_len);
   #endif
   if (can_fd < 0)
   {
      printf("error:not open the can interface\r\n");
   }
   else
   {
      pthread_mutex_lock(&can_mutex); 
      ret = write(can_fd, data, data_len);
	  pthread_mutex_unlock(&can_mutex); 
      if (ret != data_len)
      {
         printf("error:send transfer_data data_len=%d,ret=%d\r\n", data_len, ret);
      }
   }
}

void show_help(void)
{
   printf("please input test mode:\r\n");
   printf("    0: send 0000000\r\n");
   printf("    1: send 1111111\r\n");
   printf("    2: send 2222222\r\n");
}

static int open_can(void)
{
   int fd = -1, ret = 0;
   struct ifreq ifr;
   struct sockaddr_can addr;

   fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0)
    {
      printf(">>open can deivce error!\r\n");
      return -1;
    }
    else
    {
      printf(">>open can deivce success!\r\n");
    }

    strcpy(ifr.ifr_name, CAN_NAME);

    ret = ioctl(fd, SIOCGIFINDEX, &ifr);
    if (ret < 0)
    {
      printf("error:ioctl fail, erron=%d,%s\r\n", errno, strerror(errno));
      close(fd);
      fd = -1;
      return -1;
    }
    else
    {
      printf("can ifindex=%d\r\n", ifr.ifr_ifindex);
    }

    #if 0
    int canfd_on = 1;
    setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on));
    #endif
    
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(fd, (struct sockaddr *)(&addr), sizeof(addr));
    if (ret < 0)
    {
      printf(">>bind dev_handler error!\r\n");
      close(fd);
      fd = -1;
      return -1;
    }
    else
    {
	 printf("can bind ok\r\n");
    }

    return fd;
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

static OSAL_THREAD_FUNC can_recv_func(void *ptr)
{
   (void)ptr;
   
   while (1)
   {
      if (can_fd < 0)
      {
         printf("error:not open can interface\r\n");
         usleep(3000000);
      }
      else
      {
         struct can_frame recv_frames;
		 
		 pthread_mutex_lock(&can_mutex); 
         int read_len = read(can_fd, &recv_frames, sizeof(recv_frames));
		 pthread_mutex_unlock(&can_mutex); 
		 
         if (read_len != sizeof(recv_frames))
         {
            printf("error:can recv len=%d\r\n", read_len);
         }
         else
         {
            #if 1
            int ret = transfer_data((const unsigned char *)&recv_frames, sizeof(recv_frames));
	    if (ret != ERR_OK)
	    {
            	printf("transfer_data ret=%d\r\n", ret);
	    }
	    #else
	    struct timeval tv;
	    gettimeofday(&tv, NULL);
	    printf("recv data:len=%d id=0x%x time=%ld.%ld.%ld \r\n", read_len, recv_frames.can_id, tv.tv_sec, tv.tv_usec / 1000, tv.tv_usec % 1000);
	    #endif
         }
            
      }
   }
}

int main(int argc, char *argv[])
{
   int ret = 0;

   (void)argc;
   (void)argv;

#if 1

   //system("echo 1 >/sys/class/net/can0/threaded");
   
   // Set the scheduling policy to SCHED_FIFO
   struct sched_param param;
   param.sched_priority = 99; // Highest real-time priority

   // Set the scheduling policy and priority
   if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
   {
      //system("echo 950000 > /sys/fs/cgroup/cpu/user.slice/cpu.rt_runtime_us");

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

   printf("can client\r\n");

   can_fd = open_can();
   if (can_fd < 0)
   {
      printf("can't open the can interface \r\n");
      return -1;
   }

   ret = connect_server("127.0.0.1", AGV_CAN_GROUP_ID, (recv_func_cb_t)client_recv_func);
   if (ret != ERR_OK)
   {
      printf("connect server failed, ret=%d\r\n", ret);
   }
   else
   {
      printf("connect server OK!\r\n");
   }

   osal_thread_create(&can_thread_recv, 128000, &can_recv_func, NULL);

   //show_help();

   while (1)
   {
      #if 0
      int ret = 0;
      int test_mode = 0;
      int match_num = 0;

      match_num = scanf("%d", &test_mode);
      if (match_num == 1)
      {
         switch (test_mode)
         {
            case 0:
            {
               const char * send_data = "0000000";
               ret = transfer_data((const unsigned char *)send_data, strlen(send_data));
               printf("transfer_data ret=%d\r\n", ret);
               break;
            }

            case 1:
            {
               const char * send_data = "1111111";
               ret = transfer_data((const unsigned char *)send_data, strlen(send_data));
               printf("transfer_data ret=%d\r\n", ret);
               break;
            }

            case 2:
            {
               const char * send_data = "2222222";
               ret = transfer_data((const unsigned char *)send_data, strlen(send_data));
               printf("transfer_data ret=%d\r\n", ret);
               break;
            }

            default:
            {
               break;
            }
         }
      }
      #endif

      usleep(10000000);
   }

   printf("End program\n");
   return (0);
}
