/*
 * Copyright (C) 2020 InvenSense, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <inttypes.h>
#include <poll.h>
#include <unistd.h>
#include <signal.h>
#include <stdlib.h>
#include <time.h>
#include <getopt.h>
#include <math.h>
#include <string.h>
#include <pthread.h>
#include <linux/input.h>
#include "client_api.h"

//#include "ros/ros.h"
//#include "std_msgs/String.h"
//#include "sensor_msgs/Imu.h"

/* version */
#define VERSION_STR "0.0.1"

/* device files for sensor data (system dependent) */
#define DEV_NAME_ACCEL "/dev/input/event1"
#define DEV_NAME_GYRO "/dev/input/event2"
#define DEV_NAME_TEMP "/dev/input/event3"

/* all control is done by sysfs under accel */
#define SYSFS_PATH "/sys/devices/virtual/input/iam20685-accel"

/* sysfs attribute names */
#define ATTR_CHIP_NAME "name"
#define ATTR_ENABLE_ACCEL "enable_accel"
#define ATTR_ENABLE_GYRO "enable_gyro"
#define ATTR_ENABLE_TEMP "enable_temp"
#define ATTR_ODR_HZ "odr_hz"

#define PACKET_SZ_3AXIS (sizeof(struct input_event) * 6)
#define PACKET_SZ_1AXIS (sizeof(struct input_event) * 4)
#define BUF_MAX 1024

/* sensor type */
enum sensor_type
{
    SENSOR_ACCEL = 0,
    SENSOR_GYRO,
    SENSOR_TEMP,
    SENSOR_MAX
};

enum recv_flag
{
    SENSOR_ACCEL_RECV_FLAG = 1,
    SENSOR_GYRO_RECV_FLAG = 2,
    SENSOR_TEMP_RECV_FLAG = 4,
    SENSOR_MAX_RECV_FLAG = 7
};

/* sensor data */
struct sensor_data
{
    uint64_t timestamp;
    int16_t value[3];
};

struct __attribute__((packed)) sensor_data_publish
{
    int16_t accel_value[3];
    int16_t gyro_value[3];
    int16_t temp_value[3];
};

/* handler threads  */
struct handler_thread
{
    pthread_t tid; /* thread id */
    int fd[2];     /* fd for pipe */
};

/* sensor names */
static char *sensor_type_name[SENSOR_MAX] = {
    "accel", /* SENSOR_ACCEL */
    "gyro",  /* SENSOR_GYRO */
    "temp"   /* SENSOR_TEMP */
};

/* variables */
static struct handler_thread data_handler[SENSOR_MAX];
static uint64_t prev_timestamp[SENSOR_MAX];

/* commandline options */
static const struct option options[] = {
    {"help", no_argument, NULL, 'h'},
    {"accel", no_argument, NULL, 'a'},
    {"gyro", no_argument, NULL, 'g'},
    {"temp", no_argument, NULL, 't'},
    {"hz", required_argument, NULL, 'r'},
    {0, 0, 0, 0},
};

static const char *options_descriptions[] = {
    "Show this help and quit.",
    "Enable accelerometer.",
    "Enable gyroscope.",
    "Enable temperature.",
    "ODR in Hz. Common for all sensors.",
};

//sensor_msgs::Imu g_msg;
static struct sensor_data_publish g_msg; 
//ros::Publisher g_pub;
//std::mutex g_mutex;
static pthread_mutex_t g_mutex;
uint32_t g_finlish_flag = {0};
uint32_t g_num = {0};
float orientationw, orientationx, orientationy, orientationz;
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

#if 0
float invSqrt(float number)
{
	volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );

	return y;
}

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
    float sampleFreq = 100.0f;
    volatile float twoKp = 1.0f;											// 2 * proportional gain (Kp)
    volatile float twoKi = 0.0f;      
    volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
                                          // 2 * integral gain

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// 首先把加速度计采集到的值(三维向量)转化为单位向量，即向量除以模
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// 把四元数换算成方向余弦中的第三行的三个元素
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;				// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;				// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	orientationw = q0;
	orientationx = q1;
	orientationy = q2;
	orientationz = q3;
}
#endif

#if 1
void publish_msg(struct sensor_data *data, enum sensor_type type)
{
    //std::lock_guard<std::mutex> lock(g_mutex);
    pthread_mutex_lock(&g_mutex);
    switch (type)
    {
    case SENSOR_ACCEL:
    {
        //g_msg.linear_acceleration.x = (data->value[0])/32768.f*16.384*9.8;
        //g_msg.linear_acceleration.y = (data->value[1])/32768.f*16.384*9.8;
        //g_msg.linear_acceleration.z = (data->value[2])/32768.f*16.384*9.8+0.384;
        
        g_msg.accel_value[0] = data->value[0];
        g_msg.accel_value[1] = data->value[1];
        g_msg.accel_value[2] = data->value[2];
        g_finlish_flag = g_finlish_flag | SENSOR_ACCEL_RECV_FLAG;
    }
    break;
    case SENSOR_GYRO:
    {
        //g_msg.angular_velocity.x = (data->value[0])/32768.f*655*3.1415927/180;
        //g_msg.angular_velocity.y = (data->value[1])/32768.f*655*3.1415927/180;
        //g_msg.angular_velocity.z = (data->value[2])/32768.f*655*3.1415927/180;
        
        g_msg.gyro_value[0] = data->value[0];
        g_msg.gyro_value[1] = data->value[1];
        g_msg.gyro_value[2] = data->value[2];
        g_finlish_flag = g_finlish_flag | SENSOR_GYRO_RECV_FLAG;
    }
    break;
    case SENSOR_TEMP:
    {
        //g_msg.orientation.x = data->value[0];
        //g_msg.orientation.y = data->value[1];
        //g_msg.orientation.z = data->value[2];
        //g_msg.orientation.w = 0;
        
        g_msg.temp_value[0] = data->value[0];
        g_msg.temp_value[1] = data->value[1];
        g_msg.temp_value[2] = data->value[2];
        //g_msg.temp_value[3] = 0;
        
        g_finlish_flag = g_finlish_flag | SENSOR_TEMP_RECV_FLAG;
    }
    break;
    default:
        printf("error sensor_type: %d\n", type);
        break;
    }
    
    //printf("g_finlish_flag=%d \r\n", g_finlish_flag);
    
    if (SENSOR_MAX_RECV_FLAG == g_finlish_flag)
    {
    	int ret = 0;
        #if 0
        //if (ros::ok())
        {
            g_msg.header.seq = g_num++;
            g_msg.header.stamp = ros::Time::now();
            g_msg.header.frame_id = "imu_link";

            MahonyAHRSupdateIMU(0.0, 0.0, g_msg.angular_velocity.z, 0.0, 0.0, g_msg.linear_acceleration.z);

            g_msg.orientation.w = orientationw;
            g_msg.orientation.x = 0.0;
            g_msg.orientation.y = 0.0;
            g_msg.orientation.z = orientationz;

            g_msg.orientation_covariance[0] = 1e6;
	        g_msg.orientation_covariance[4] = 1e6;
	        g_msg.orientation_covariance[8] = 1e-6;

            g_msg.angular_velocity_covariance[0] = 1e6;
	        g_msg.angular_velocity_covariance[4] = 1e6;
	        g_msg.angular_velocity_covariance[8] = 1e-6;
            
            //g_pub.publish(g_msg);
            //ros::spinOnce();
            // printf("publish msg\n");
        }
        #endif
        
        ret = transfer_data((const unsigned char *)&g_msg, sizeof(g_msg));
        if (ret != ERR_OK)
        {
            printf("transfer_data ret=%d\r\n", ret);
        }
        
        g_finlish_flag = 0;
    }
    
    pthread_mutex_unlock(&g_mutex);
}
#endif


/* get the current time */
int64_t get_current_timestamp(void)
{
    struct timespec tp;

    clock_gettime(CLOCK_MONOTONIC, &tp);
    return (int64_t)tp.tv_sec * 1000000000LL + (int64_t)tp.tv_nsec;
}

/* write a value to sysfs */
static int write_sysfs_int(char *sysfs_path, char *attr, int data)
{
    FILE *fp;
    int ret;
    char path[1024];

    ret = snprintf(path, sizeof(path), "%s/%s", sysfs_path, attr);
    if (ret < 0 || ret >= (int)sizeof(path))
    {
        return -1;
    }

    ret = 0;
    printf("sysfs: %d -> %s\n", data, path);
    fp = fopen(path, "w");
    if (fp == NULL)
    {
        ret = -errno;
        printf("Failed to open %s\n", path);
    }
    else
    {
        if (fprintf(fp, "%d\n", data) < 0)
        {
            printf("Failed to write to %s\n", path);
            ret = -errno;
        }
        fclose(fp);
    }
    fflush(stdout);
    return ret;
}

/* get chip name from sysfs */
static int show_chip_name(char *sysfs_path)
{
    FILE *fp;
    int ret;
    char name[256];
    char path[1024];

    ret = snprintf(path, sizeof(path), "%s/%s", sysfs_path, ATTR_CHIP_NAME);
    if (ret < 0 || ret >= (int)sizeof(path))
    {
        return -1;
    }

    ret = 0;
    fp = fopen(path, "r");
    if (fp == NULL)
    {
        ret = -errno;
        printf("Failed to open %s\n", path);
    }
    else
    {
        if (fscanf(fp, "%s", name) != 1)
        {
            printf("Failed to read chip name\n");
            ret = -1;
        }
        else
            printf("chip : %s\n", name);
        fclose(fp);
    }
    fflush(stdout);
    return ret;
}

/* enable sensor through sysfs */
static int enable_sensor(enum sensor_type type, bool en)
{
    int ret = 0;

    if (type == SENSOR_ACCEL)
    {
        ret = write_sysfs_int(SYSFS_PATH, ATTR_ENABLE_ACCEL, en);
    }
    else if (type == SENSOR_GYRO)
    {
        ret = write_sysfs_int(SYSFS_PATH, ATTR_ENABLE_GYRO, en);
    }
    else if (type == SENSOR_TEMP)
    {
        ret = write_sysfs_int(SYSFS_PATH, ATTR_ENABLE_TEMP, en);
    }
    else
    {
        printf("invalid sensor type\n");
    }
    fflush(stdout);
    return ret;
}

/* set odr through sysfs */
static int set_sensor_rate(int hz)
{
    int ret = 0;

    ret = write_sysfs_int(SYSFS_PATH, ATTR_ODR_HZ, hz); /* any sensor type can be used */

    return ret;
}

/* show usage */
void usage(void)
{
    unsigned int i;

    printf("Usage:\n\t test-sensors-inputsub [-a] [-g] [-t] [-r]"
           "\n\nOptions:\n");
    for (i = 0; options[i].name; i++)
        printf("\t-%c, --%s\n\t\t\t%s\n",
               options[i].val, options[i].name,
               options_descriptions[i]);
    printf("Version:\n\t%s\n", VERSION_STR);
    fflush(stdout);
}

/* signal handler to disable sensors when ctr-C */
static void sig_handler(int s)
{
    int ret = 0;
    int i;

    (void)s;

    /* disable all sensors */
    for (i = 0; i < SENSOR_MAX; i++)
    {
        printf("Disable %s\n", sensor_type_name[i]);
        ret = enable_sensor((enum sensor_type)(i), false);
        if (ret)
        {
            printf("failed to disable %s\n", sensor_type_name[i]);
            fflush(stdout);
        }
    }

    for (i = 0; i < SENSOR_MAX; i++)
    {
        pthread_kill(data_handler[i].tid, SIGTERM);
    }

    fflush(stdout);
    exit(1);
}

/* get one packet */
static int get_one_packet(struct input_event *event, struct sensor_data *data, enum sensor_type type)
{
    uint32_t sec, nsec;

    if (type >= SENSOR_MAX)
        return -EINVAL;

    /* packet format
     *  <event>   <code>         <value>
     *  EV_MSC    6              timestamp (tv_sec)
     *  EV_MSC    6              timestamp (tv_nsec)
     *  EV_MSC    MSC_GESTURE    x or temp
     *  EV_MSC    MSC_RAW        y (not populated for temp)
     *  EV_MSC    MSC_SCAN       z (not populated for temp)
     *  EV_SYNC   SYN_REPORT
     */

    /* EV_MSC, 6, tv_sec */
    if (event->type == EV_MSC && event->code == 6)
        sec = (uint32_t)event->value;
    else
        return -EINVAL;

    /* EV_MSC, 6, tv_nsec */
    event++;
    if (event->type == EV_MSC && event->code == 6)
        nsec = (uint32_t)event->value;
    else
        return -EINVAL;

    /* generate timestamp */
    data->timestamp = (uint64_t)sec * 1000000000 + (uint64_t)nsec;

    /* EV_MSC, MSC_GESTURE, x */
    event++;
    if (event->type == EV_MSC && event->code == MSC_GESTURE)
        data->value[0] = (int16_t)event->value;
    else
        return -EINVAL;

    if (type == SENSOR_TEMP)
    {
        /* temp data is one axis */
        data->value[1] = 0;
        data->value[2] = 0;
        return 0;
    }

    /* EV_MSC, MSC_RAW, y */
    event++;
    if (event->type == EV_MSC && event->code == MSC_RAW)
        data->value[1] = (int16_t)event->value;
    else
        return -EINVAL;

    /* EV_MSC, MSC_SCAN, z */
    event++;
    if (event->type == EV_MSC && event->code == MSC_SCAN)
        data->value[2] = (int16_t)event->value;
    else
        return -EINVAL;

    /* EV_SYNC */
    event++;
    if (event->type != EV_SYN || event->code != SYN_REPORT)
        return -EINVAL;

    return 0;
}

/* handle data packet */
static void handle_one_packet(struct sensor_data *data, enum sensor_type type)
{
    /* TODO:
     * Integrate a function here according to the requirement on own platform.
     * The below is to print data for test.
     */
    #if 0
    if (type == SENSOR_ACCEL)
    {
         printf("Accel (LSB)  , %+6d, %+6d, %+6d, %20" PRId64 ", %8.3f, %8.3f\n",
                data->value[0], data->value[1], data->value[2],
                data->timestamp,
                (float)(data->timestamp - prev_timestamp[type]) / 1000000.f,
                (float)(get_current_timestamp() - data->timestamp) / 1000000.f);
    }
    else if (type == SENSOR_GYRO)
    {
         printf("Gyro  (LSB)  , %+6d, %+6d, %+6d, %20" PRId64 ", %8.3f, %8.3f\n",
                data->value[0], data->value[1], data->value[2],
                data->timestamp,
                (float)(data->timestamp - prev_timestamp[type]) / 1000000.f,
                (float)(get_current_timestamp() - data->timestamp) / 1000000.f);
    }
    else if (type == SENSOR_TEMP)
    {
         printf("Temp  (LSB)  , %+6d, %+6d, %+6d, %20" PRId64 ", %8.3f, %8.3f\n",
                data->value[0], data->value[1], data->value[2],
                data->timestamp,
                (float)(data->timestamp - prev_timestamp[type]) / 1000000.f,
                (float)(get_current_timestamp() - data->timestamp) / 1000000.f);
    }
    #endif
    
    prev_timestamp[type] = data->timestamp;

    publish_msg(data, type);
}

/* data handler */
void *accel_handler(void *arg)
{
    int ret;
    struct sensor_data data = {0};
    char buf[PACKET_SZ_3AXIS];

    (void)arg;

    while (1)
    {
        ret = read(data_handler[SENSOR_ACCEL].fd[0], buf, PACKET_SZ_3AXIS);
        if (ret == PACKET_SZ_3AXIS)
        {
            ret = get_one_packet((struct input_event *)buf, &data, SENSOR_ACCEL);
            if (ret)
            {
                printf("failed to parse packet for accel\n");
                fflush(stdout);
            }
            handle_one_packet(&data, SENSOR_ACCEL);
        }
    }
}

void *gyro_handler(void *arg)
{
    int ret;
    struct sensor_data data = {0};
    char buf[PACKET_SZ_3AXIS];

    (void)arg;

    while (1)
    {
        ret = read(data_handler[SENSOR_GYRO].fd[0], buf, PACKET_SZ_3AXIS);
        if (ret == PACKET_SZ_3AXIS)
        {
            ret = get_one_packet((struct input_event *)buf, &data, SENSOR_GYRO);
            if (ret)
            {
                printf("failed to parse packet for gyro\n");
                fflush(stdout);
            }
            handle_one_packet(&data, SENSOR_GYRO);
        }
    }
}

void *temp_handler(void *arg)
{
    int ret;
    struct sensor_data data = {0};
    char buf[PACKET_SZ_1AXIS];

    (void)arg;

    while (1)
    {
        ret = read(data_handler[SENSOR_TEMP].fd[0], buf, PACKET_SZ_1AXIS);
        if (ret == PACKET_SZ_1AXIS)
        {
            ret = get_one_packet((struct input_event *)buf, &data, SENSOR_TEMP);
            if (ret)
            {
                printf("failed to parse packet for temp\n");
                fflush(stdout);
            }
            handle_one_packet(&data, SENSOR_TEMP);
        }
    }
}

static void client_recv_func(unsigned char * data, int data_len)
{
   //int ret = 0;
   (void)data;
   (void)data_len;
   #ifdef PRINT_DATA
   print_data(data, data_len);
   #endif
   
}

/* main */
int main(int argc, char *argv[])
{
    int ret;
    int i;
    struct sigaction sig_action;
    //int opt, option_index;
    int fd[SENSOR_MAX];
    struct pollfd fds[SENSOR_MAX];
    char buf[BUF_MAX];
    ssize_t num;
    bool accel_en = true;
    bool gyro_en = true;
    bool temp_en = true;
    unsigned long odr_hz = 100;
    
    (void)argc;
    (void)argv;

    #if 0
    while ((opt = getopt_long(argc, argv, "hagtcr:", options, &option_index)) != -1)
    {
        switch (opt)
        {
        case 'a':
            accel_en = true;
            break;
        case 'g':
            gyro_en = true;
            break;
        case 't':
            temp_en = true;
            break;
        case 'r':
            odr_hz = strtoul(optarg, NULL, 10);
            break;
        case 'h':
            usage();
            return 0;
        }
    }
    if (!accel_en && !gyro_en && !temp_en)
    {
        usage();
        return 0;
    }
    #endif
    
   ret = connect_server("127.0.0.1", J5_IMU_GROUP_ID, (recv_func_cb_t)client_recv_func);
   if (ret != ERR_OK)
   {
      printf("connect server failed, ret=%d\r\n", ret);
   }
   else
   {
      printf("connect server OK!\r\n");
   }

    /* signal handling */
    sig_action.sa_handler = sig_handler;
    sigemptyset(&sig_action.sa_mask);
    sig_action.sa_flags = 0;
    sigaction(SIGINT, &sig_action, NULL);
    
    pthread_mutex_init(&g_mutex, NULL);

    printf(">Start\n");
    fflush(stdout);

    /* crate pipe */
    for (i = 0; i < SENSOR_MAX; i++)
    {
        ret = pipe(data_handler[i].fd);
        if (ret)
        {
            return ret;
        }
    }

    //ros::init(argc, argv, "talker");
    //ros::NodeHandle nh;
    //g_pub = nh.advertise<sensor_msgs::Imu>("imu_20685", 1000);

    /* crate thread */
    pthread_create(&data_handler[SENSOR_ACCEL].tid, NULL, accel_handler, NULL);
    pthread_create(&data_handler[SENSOR_GYRO].tid, NULL, gyro_handler, NULL);
    pthread_create(&data_handler[SENSOR_TEMP].tid, NULL, temp_handler, NULL);

    /* show chip name */
    ret = show_chip_name(SYSFS_PATH);
    if (ret)
    {
        return ret;
    }

    /* make sure all sensors are disabled */
    printf(">Disable accel\n");
    fflush(stdout);
    ret = enable_sensor(SENSOR_ACCEL, 0);
    if (ret)
    {
        printf("failed to enable accel\n");
        fflush(stdout);
        return ret;
    }
    printf(">Disable gyro\n");
    fflush(stdout);
    ret = enable_sensor(SENSOR_GYRO, 0);
    if (ret)
    {
        printf("failed to enable gyro\n");
        fflush(stdout);
        return ret;
    }

    /* odr */
    printf(">Set ODR\n");
    fflush(stdout);
    ret = set_sensor_rate(odr_hz);
    if (ret)
    {
        printf("failed to set ODR\n");
        fflush(stdout);
        return ret;
    }

    /* enable sensors */
    printf(">Enable Accel %d\n", accel_en);
    fflush(stdout);
    if (accel_en)
    {
        ret = enable_sensor(SENSOR_ACCEL, true);
        if (ret)
        {
            printf("failed to enable Accel\n");
            fflush(stdout);
            return ret;
        }
    }
    printf(">Enable Gyro %d\n", gyro_en);
    fflush(stdout);
    if (gyro_en)
    {
        ret = enable_sensor(SENSOR_GYRO, true);
        if (ret)
        {
            printf("failed to enable Gyro\n");
            fflush(stdout);
            return ret;
        }
    }
    printf(">Enable Temp %d\n", temp_en);
    fflush(stdout);
    if (temp_en)
    {
        ret = enable_sensor(SENSOR_TEMP, true);
        if (ret)
        {
            printf("failed to enable Temp\n");
            fflush(stdout);
            return ret;
        }
    }

    /* input devices */
    /* accel */
    fd[SENSOR_ACCEL] = open(DEV_NAME_ACCEL, O_RDWR | O_NONBLOCK);
    if (fd[SENSOR_ACCEL] < 0)
    {
        printf("Failed to open %s\n", DEV_NAME_ACCEL);
        return -1;
    }

    /* gyro */
    fd[SENSOR_GYRO] = open(DEV_NAME_GYRO, O_RDWR | O_NONBLOCK);
    if (fd[SENSOR_GYRO] < 0)
    {
        printf("Failed to open %s\n", DEV_NAME_GYRO);
        return -1;
    }

    /* temp */
    fd[SENSOR_TEMP] = open(DEV_NAME_TEMP, O_RDWR | O_NONBLOCK);
    if (fd[SENSOR_TEMP] < 0)
    {
        printf("Failed to open %s\n", DEV_NAME_TEMP);
        return -1;
    }

    /* dummy read */
    ret = read(fd[SENSOR_ACCEL], buf, sizeof(buf));
    ret = read(fd[SENSOR_GYRO], buf, sizeof(buf));
    ret = read(fd[SENSOR_TEMP], buf, sizeof(buf));

    /* prepare poll */
    fds[SENSOR_ACCEL].fd = fd[SENSOR_ACCEL];
    fds[SENSOR_ACCEL].events = POLLPRI | POLLIN;
    fds[SENSOR_ACCEL].revents = 0;

    fds[SENSOR_GYRO].fd = fd[SENSOR_GYRO];
    fds[SENSOR_GYRO].events = POLLPRI | POLLIN;
    fds[SENSOR_GYRO].revents = 0;

    fds[SENSOR_TEMP].fd = fd[SENSOR_TEMP];
    fds[SENSOR_TEMP].events = POLLPRI | POLLIN;
    fds[SENSOR_TEMP].revents = 0;

    /* poll loop */
    printf("start\n");
    while (1)
    {
        ret = poll(fds, SENSOR_MAX, -1);
        if (ret > 0)
        {
            /* accel */
            if (fds[SENSOR_ACCEL].events & POLLIN)
            {
                num = read(fd[SENSOR_ACCEL], buf, sizeof(buf));
                if (num > 0)
                {
                    /* store data to buffer */
                    ret = write(data_handler[SENSOR_ACCEL].fd[1], buf, num);
                    if (ret < 0)
                    {
                        return ret;
                    }
                }
            }
            /* gyro */
            if (fds[SENSOR_GYRO].events & POLLIN)
            {
                num = read(fd[SENSOR_GYRO], buf, sizeof(buf));
                if (num > 0)
                {
                    /* store data to buffer */
                    ret = write(data_handler[SENSOR_GYRO].fd[1], buf, num);
                    if (ret < 0)
                    {
                        return ret;
                    }
                }
            }
            /* temp */
            if (fds[SENSOR_TEMP].events & POLLIN)
            {
                num = read(fd[SENSOR_TEMP], buf, sizeof(buf));
                if (num > 0)
                {
                    /* store data to buffer */
                    ret = write(data_handler[SENSOR_TEMP].fd[1], buf, num);
                    if (ret < 0)
                    {
                        return ret;
                    }
                }
            }
        }
    }

    return 0;
}


