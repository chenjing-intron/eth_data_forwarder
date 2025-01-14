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

void client_recv_func(unsigned char * data, int data_len)
{
   print_data(data, data_len);
}

static void show_help(void)
{
   printf("please input test mode:\r\n");
   printf("    0: send 0000000\r\n");
   printf("    1: send 1111111\r\n");
   printf("    2: send 2222222\r\n");
}

int main(int argc, char *argv[])
{
   int ret = 0;

   (void)argc;
   (void)argv;

   printf("client\r\n");

   ret = connect_server("127.0.0.1", (recv_func_cb_t)client_recv_func);
   if (ret != ERR_OK)
   {
      printf("connect server failed, ret=%d\r\n", ret);
   }
   else
   {
      printf("connect server OK!\r\n");
   }

   show_help();

   while (1)
   {
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

      //usleep(1000000);
   }

   printf("End program\n");
   return (0);
}
