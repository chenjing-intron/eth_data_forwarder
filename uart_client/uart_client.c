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
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include "client_api.h"
#include <time.h>
#include <errno.h>


#include <pthread.h>

#define OSAL_THREAD_HANDLE pthread_t *
#define OSAL_THREAD_FUNC void
#define OSAL_THREAD_FUNC_RT void

static int uart_fd = -1;

static OSAL_THREAD_HANDLE uart_thread_recv;

#define PRINT_DATA

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

// 设置串口参数
int setup_uart(int fd, int baud_rate) {
    struct termios tty;

    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        return -1;
    }

    // 设置波特率
    cfsetospeed(&tty, baud_rate);
    cfsetispeed(&tty, baud_rate);

    // 设置8位数据位，无奇偶校验，1位停止位
    tty.c_cflag &= ~PARENB; // 无奇偶校验
    tty.c_cflag &= ~CSTOPB; // 1位停止位
    tty.c_cflag &= ~CSIZE;  // 清除数据位掩码
    tty.c_cflag |= CS8;     // 8位数据位

    // 禁用硬件流控制
    tty.c_cflag &= ~CRTSCTS;

    // 启用接收
    tty.c_cflag |= CREAD | CLOCAL;

    // 禁用规范模式（原始模式）
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 禁用软件流控制
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    // 设置原始输入模式
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 设置原始输出模式
    tty.c_oflag &= ~OPOST;

    // 设置超时和最小字符数
    tty.c_cc[VMIN] = 1;   // 至少读取1个字符
    tty.c_cc[VTIME] = 10; // 等待时间为1秒

    // 应用设置
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return -1;
    }

    return 0;
}


void client_recv_func(unsigned char * data, int data_len)
{
   //int ret = 0;

   #ifdef PRINT_DATA
   print_data(data, data_len);
   #endif
   
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

static OSAL_THREAD_FUNC uart_recv_func(void *ptr)
{
   (void)ptr;
   
   while (1)
   {
      if (uart_fd < 0)
      {
         printf("error:not open can interface\r\n");
         usleep(3000000);
      }
      else
      {
	 int ret = 0;
	 #define BUF_SIZE (512)
	 char buf[BUF_SIZE];
	 fd_set read_fds;
	 // 读取数据
	 FD_ZERO(&read_fds);
         FD_SET(uart_fd, &read_fds);

        // 使用 select 监听串口
        ret = select(uart_fd + 1, &read_fds, NULL, NULL, NULL);
        if (ret == -1) {
            perror("select");
            break;
        }

        // 检查是否有数据可读
        if (FD_ISSET(uart_fd, &read_fds)) {
            ssize_t n = read(uart_fd, buf, BUF_SIZE - 1);
            if (n > 0) {
                //buf[n] = '\0'; // 确保字符串以 null 结尾
                //printf("Received %ld bytes: %s\n", n, buf);
		ret = transfer_data((const unsigned char *)buf, n);
            	if (ret != ERR_OK)
            	{
                	printf("transfer_data ret=%d\r\n", ret);
            	}
            } else if (n == 0) {
                printf("Connection closed\n");
                //break;
            } else {
                perror("read");
                //break;
            }
        }
      }
   }
}

int main(int argc, char *argv[])
{
   int ret = 0;
   
   if (argc != 3) {
        fprintf(stderr, "Usage: %s <uart_device> <baud_rate>\n", argv[0]);
        exit(EXIT_FAILURE);
    }

    char *uart_device = argv[1];
    int baud_rate = atoi(argv[2]);

    // 打开UART设备
    uart_fd = open(uart_device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (uart_fd == -1) {
        perror("open");
        exit(EXIT_FAILURE);
    }

    // 设置UART参数
    if (setup_uart(uart_fd, baud_rate) == -1) {
        close(uart_fd);
        exit(EXIT_FAILURE);
    }

   ret = connect_server("127.0.0.1", AGV_UART_GROUP_ID, (recv_func_cb_t)client_recv_func);
   if (ret != ERR_OK)
   {
      printf("connect server failed, ret=%d\r\n", ret);
   }
   else
   {
      printf("connect server OK!\r\n");
   }

   osal_thread_create(&uart_thread_recv, 128000, &uart_recv_func, NULL);

   while (1)
   {
      usleep(10000000);
   }

   printf("End program\n");
   return (0);
}
