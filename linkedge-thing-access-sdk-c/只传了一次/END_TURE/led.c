/*
 * Copyright (c) 2014-2018 Alibaba Group. All rights reserved.
 * License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <cJSON.h>
#include <time.h>
#include "log.h"
#include "le_error.h"
#include "leda.h"
#include <sys/mman.h>
#include <termios.h> 
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <sys/types.h>
#include <linux/sysctl.h>
#include <syslog.h>
#include <stdbool.h>
#include <sys/time.h>
#ifdef __cplusplus
extern "C" {
#endif
/*
  struct  tm  
  {  
      int tm_sec; //  Seconds:  0-59  (K&R  says  0-61?)  
      int tm_min; //  Minutes:  0-59   
      int tm_hour; // Hours  since  midnight:  0-23  
      int tm_mday; //  Day  of  the  month:  1-31  
      int tm_mon; // Months  *since*  january:  0-11  
      int tm_year; //  Years  since  1900    
      int tm_wday; //  Days  since  Sunday  (0-6)  
      int tm_yday;   //Days  since  Jan.  1:  0-365   
      int tm_isdst;  // +1  Daylight  Savings  Time,  0  No  DST,  
       				//  -1  don't  know  
  };  
*/
#define LED_TAG_NAME            "SNAI_led"
#define MAX_DEVICE_NUM          32
leda_device_data_t dev_proper_data[48+2] = {
                {
                    .type  = LEDA_TYPE_FLOAT,
                    .key   = {"temperature"},
                    .value = {"-100"}
                }
};

static int g_dev_handle_list[MAX_DEVICE_NUM] = 
{
    INVALID_DEVICE_HANDLE
};
static int  g_dev_handle_count = 0;
/////////////////////////////////////////////////////////////////////////

#define SNAI_DEBUG_INFO(fmt, args...) if(SNAI_TEMP_log != NULL)									\
									{															\
										SNAI_cur_time = time(NULL);								\
										memset(SNAI_TEMP_log, 0, sizeof(char)*2048);			\
										SNAI_get_timestamp(SNAI_buf_date, 20, SNAI_cur_time);	\
										sprintf(SNAI_TEMP_log,									\
										"LOG_%s-FILE:%s-%s(%d)--INFOR:"#fmt"\n", 				\
										SNAI_buf_date,__FILE__,__FUNCTION__,__LINE__,##args);	\
										SNAI_log_ww(SNAI_TEMP_log);								\
									}
leda_device_data_t dev_event_data[48+1] = {
                {
                    .type  = LEDA_TYPE_ENUM,
                    .key   = {"error"},
                    .value = {"0"}//无故障//1传感器故障
                }
};
typedef struct SNAI_RS485_HANDLE_NUM
{
    device_handle_t  SNAI_485dev_handle[24+1];
    int              Parameter_count[24+1];
    int              Parameter_ptr[24+1];
    bool             SNAI_device_ready[24+1];
    bool             SNAI_DEVICE_EXIST[24+1];
}SNAI_RS485_HANDLE_NUM_t;
SNAI_RS485_HANDLE_NUM_t SNAI_ALL_DEVICE_REPORT;
typedef struct SNAI_OLD_DATA
{
   float SNAI_RS485_vTemp[4];
   float SNAI_RS485_vHumi[4];
   unsigned short SNAI_RS485_vFY;
   unsigned short SNAI_RS485_vCo2;
   unsigned short SNAI_RS485_vCo;
   unsigned short SNAI_RS485_vNH3;
   unsigned short SNAI_RS485_vWind_D;
   unsigned short SNAI_RS485_vWind_S;
   unsigned short SNAI_RS485_vPosition_L;
   unsigned short SNAI_RS485_vPosition_R;
   unsigned short SNAI_RS485_viLLumination;
   double SNAI_RS485_vWater_meter;
   float SNAI_RS485_Boiler;
}SNAI_OLD_DATA_t;
/* TYPE为传感器类型，0xA0-温度，0xA1-湿度，0xA2-氨气，0xA3-CO2，0xA4-CO，0xA5-光照，0xA6-水表，0xA7-负压，0xA8-风向，0xA9-风速，
0xAA-室外温度，0xAB-室外湿度，【0xAC-锅炉水温，0xC3-温湿度，0xC4-室外 温湿度传感器程序内部代号】
 */
#define SNAI_MSG_TYPE_DATA          0xFF
#define SNAI_MSG_TYPE_DATA2         0xEE
#define SNAI_READ_DATA              0x52
#define SNAI_READ_ADDR              0x51
#define SNAI_USART_RX_ONE_TIME      256
#define SNAI_MSG_HEADER_LEN         0x02
#define SNAI_MSG_MIN_LEN            0x08   /* msg_head_len  + tlv_len_min + crc16_len */

#define SNAI_TYPE_tmpt              0xA0/*2/3/4*/
#define SNAI_TYPE_humi              0xA1/*2/3/4*/
#define SNAI_TYPE_NH3               0xA2/*9*/
#define SNAI_TYPE_Co2               0xA3/*8*/
#define SNAI_TYPE_Co                0xA4/*NULL*/
#define SNAI_TYPE_Illumination      0xA5/*10*/
#define SNAI_TYPE_Water_Meter       0xA6/*17*/
#define SNAI_TYPE_Negative_Pressure 0xA7/*7*/
#define SNAI_TYPE_Wind_Direction    0xA8
#define SNAI_TYPE_Wind_Speed        0xA9
#define SNAI_TYPE_out_tmpt          0xAA/*5*/
#define SNAI_TYPE_out_humi          0xAB/*5*/
#define SNAI_TYPE_Boiler            0xAC/*6*/
#define SNAI_TYPE_Position_Left     0xAD/*11*/
#define SNAI_TYPE_Position_Right    0xAE/*12*/
#define SNAI_TYPE_Other             0xAF

char SNAI_TEMP_log[2048] = {0};	
unsigned char SNAI_CRC_value_L = 0;
unsigned char SNAI_CRC_value_H = 0;
unsigned short SNAI_CRC_value = 0;//2个字节长度
static int SNAI_file_id = -1;
static int SNAI_log_id = -1;
pthread_mutex_t SNAI_Decode_mutex_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t SNAI_cond = PTHREAD_COND_INITIALIZER;
//unsigned char buf[SNAI_USART_RX_ONE_TIME];
unsigned char SNAI_all_device_value[15]={0};

char SNAI_buf_date[20] = {0};
time_t SNAI_cur_time;
int SNAI_Origin_Day = -1;
typedef struct SNAI_pthread_opt
{
    void* (*pthread_opt_func)(void *data);
    void*  data;
    pthread_t pthread_id;
}SNAI_pthread_opt_user;
typedef struct SNAI_circular_buffer
{
    void *ptr;
    unsigned long count;
    unsigned long read_offset;
    unsigned long write_offset;
} SNAI_circular_buffer;

SNAI_pthread_opt_user* opt_seq_ptr = NULL;
unsigned long cb_bytes_can_read(SNAI_circular_buffer *cb);
SNAI_circular_buffer *cb_create(unsigned long order);

unsigned short CRC_Return(unsigned char *Crc_Buf, unsigned char Crc_Len);
static unsigned short Crc_Cal(unsigned short Data, unsigned short GenPoly, unsigned short CrcData);
void pthread_opt_seq_kill(SNAI_pthread_opt_user *op);
int pthread_opt_seq_exec(SNAI_pthread_opt_user *op);
void main_thread_hander(int signo);

unsigned long cb_bytes_can_read(SNAI_circular_buffer *cb);
void cb_read_offset_inc(SNAI_circular_buffer *cb, unsigned long  cnt);
void cbClear(SNAI_circular_buffer *cb);
int usart_init(void);
void usart_cleanup(void);
int usart_tx(unsigned char* msg , int len);
int usart_rx(SNAI_circular_buffer *cb, int len);
void usart_discard(SNAI_circular_buffer *cb);
void usart_sig_hander(int signo);
void* usart_rx_start(void* data);
void decode_sig_hander(int signo);
void tlv_decode(SNAI_circular_buffer *cb);
int SNAI_log_init(void);
void SNAI_log_ww(char *str);
void* status_report(void* data);
void SNAI_driver_exit(int xx);
void* TX_READ(void* data);
static char *SNAI_get_timestamp(char *buf, int len, time_t cur_time);
void SNAI_DEVICE_RS485_ADDR_HANDLE(device_handle_t SNAI_handle,char *RS485_ADDR);

/////////////////////////////////////////////////////////////////////////
void pthread_opt_seq_kill(SNAI_pthread_opt_user *op)
{
	int i = 0;
	while(op[i].pthread_opt_func!=NULL)
	{
		pthread_kill(op[i].pthread_id, SIGKILL);
		i++;
	}
}
static char *SNAI_get_timestamp(char *buf, int len, time_t cur_time)
{
    struct tm tm_time;
    localtime_r(&cur_time, &tm_time);

    snprintf(buf, len, "%d-%u-%d-%d:%d:%d",
             1900 + tm_time.tm_year, 1 + tm_time.tm_mon,
             tm_time.tm_mday, tm_time.tm_hour,
             tm_time.tm_min, tm_time.tm_sec);
        if(SNAI_Origin_Day != tm_time.tm_mday && tm_time.tm_hour == 0)SNAI_log_init();
	
    return buf;
}
int pthread_opt_seq_exec(SNAI_pthread_opt_user *op)
{
	int i = 0;
	int err = 0;
	while(op[i].pthread_opt_func!=NULL)//循环检查需要执行的线程函数队列，函数指针是否为空
	{
		err = pthread_create(&op[i].pthread_id,NULL,op[i].pthread_opt_func, op[i].data);
		if(err)
			break;//如果执行错误，则跳出while循环
		i++ ;
	}
	i--;
	SNAI_DEBUG_INFO("The create ok!");
	printf("The create ok!\r\n");
	while(i >= 0)
	{
		err = pthread_join(op[i].pthread_id, NULL) ;
		if(err)
			break;
		i-- ;
	}
		SNAI_DEBUG_INFO("The process over!");
    printf("The process over!\r\n");
	if(err)
		pthread_opt_seq_kill(op);

	return  err;
}
void main_thread_hander(int signo)
{
	SNAI_DEBUG_INFO("game over!");
	printf("game over!\r\n");
	//pthread_mutex_destroy(&SNAI_Decode_mutex_lock);
	pthread_opt_seq_kill(opt_seq_ptr);
}
void SNAI_driver_exit(int xx)
{
	 /* 退出驱动 */
    log_i(LED_TAG_NAME, "demo exit\r\n");
		SNAI_DEBUG_INFO("退出驱动！！");
		pthread_exit(0);
}
SNAI_circular_buffer *cb_create(unsigned long order)
{

	int fd = 0, status = 0;
	void *address = NULL;
	char path[] = "/dev/shm/ibsd_cache_XXXXXX";
	SNAI_circular_buffer *cb = (SNAI_circular_buffer *)malloc(sizeof(SNAI_circular_buffer));//分配一个新的内存空间。cb_struct_pointer

	if (NULL == cb)
	{
		return NULL;
	}

	order = (order <= 13 ? 13 : order);
	cb->count = 1UL << order;//左移order位,1UL表示无符号长整形1 左移13次 10000000000000=8192=8KB
	cb->read_offset = 0;
	cb->write_offset = 0;

	cb->ptr = mmap(NULL, cb->count+(cb->count>>1), PROT_READ | PROT_WRITE, MAP_ANONYMOUS |MAP_SHARED, -1, 0);
	if (MAP_FAILED == cb->ptr)
	{
		SNAI_DEBUG_INFO("map failed ");
		printf("map failed \n");
		abort();
	}

	fd = mkstemp(path);
	if (0 > fd)
	{
		SNAI_DEBUG_INFO("mkstemp failed");
		printf("mkstemp failed \n");
		abort();
	}

	/*
	status = unlink(path);
	if (0 != status)
	{
		printf("unlink failed \n");
		abort();
	}*/

	status = ftruncate(fd, cb->count);
	if (0 != status)
	{
		SNAI_DEBUG_INFO("ftruncate failed");
		printf("ftruncate failed \n");
		abort();
	}

	address = mmap(cb->ptr , cb->count, PROT_READ | PROT_WRITE, MAP_FIXED | MAP_SHARED, fd, 0);
	if (address != cb->ptr)
	{
		SNAI_DEBUG_INFO("map1 failed");
		printf("map1 failed \n");
		printf("errno = %d \n" , errno);
		abort();
	}

	address = mmap(cb->ptr + cb->count, cb->count>>2, PROT_READ | PROT_WRITE, MAP_FIXED | MAP_SHARED, fd, 0);
	if (address != cb->ptr + cb->count)
	{
		SNAI_DEBUG_INFO("map2 failed");
		printf("map2 failed \n");
		printf("errno = %d \n" , errno);
		abort();
	}

	status = close(fd);
	if (0 != status)
	{
		SNAI_DEBUG_INFO("close failed");
		printf("close failed \n");
		abort();
	}

	return cb;
}
unsigned long cb_bytes_can_read(SNAI_circular_buffer *cb)
{
    return cb->write_offset - cb->read_offset;
}
void cb_read_offset_inc(SNAI_circular_buffer *cb, unsigned long  cnt)
{
	cb->read_offset += cnt ;//读取统计地址+1
	if (cb->read_offset > cb->count)//大于8K则说明读到顶
	{
		cb->read_offset -= cb->count;//归零
		cb->write_offset -= cb->count;//归零
	}
}
void cbClear(SNAI_circular_buffer *cb)
{
	cb->read_offset = 0;
	cb->write_offset = 0;
}
/* 串口初始化函数 */
int usart_init(void)
{
	if ((SNAI_file_id = open("/dev/485_serial", O_RDWR |  O_NOCTTY |  O_NDELAY))<0)//打开串口1
	{
		perror("USART: Failed to open the usart port! \n");//该函数将错误消息打印到流stderr错误流
		SNAI_DEBUG_INFO("未能打开串口！Please check the Serialport connect...");
		SNAI_DEBUG_INFO("Please check the Serialport Device【485_serial】");
		return -1;
	}
	struct termios options; // the termios structure is vital//串口配置结构体
	tcgetattr(SNAI_file_id, &options); // sets the parameters associated with file 将第一个的状态输入第二个参数

	//Set To Raw Mode
	options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*///输入模式标志
	options.c_oflag  &= ~OPOST;   /*Output*///输出模式标志
	//控制模式标志
	// 9600 baud, 8-bit, enable receiver, no modem control lines 9600波特，8位，使能接收器，无调制解调器控制线
	options.c_cflag = B9600 | CS8 | CREAD | CLOCAL;//与协调器通讯波特率
	//本地模式标志
	options.c_iflag = IGNPAR | ICRNL; // ignore partity errors, CR -> newline

	tcflush(SNAI_file_id, TCIFLUSH); // discard file information not transmitted刷清输入、输出队列
	tcsetattr(SNAI_file_id, TCSANOW, &options); // changes occur immmediately立即执行而不等待数据发送或者接受完成。

	return  1;
}
int SNAI_log_init(void)
{
	char SNAI_LOG_File_TIME[40];
	struct tm  *tp;  
    time_t t = time(NULL); 
    tp = localtime(&t);
	SNAI_Origin_Day = tp->tm_mday;//首次 
    sprintf(SNAI_LOG_File_TIME,"/dev/shm/SNAI_LOG-%d-%u-%d-%d-%d",tp->tm_year+1900,tp->tm_mon+1,tp->tm_mday 
   			,tp->tm_hour,tp->tm_min); 
	if((SNAI_log_id = open(SNAI_LOG_File_TIME,O_CREAT | O_RDWR | O_APPEND | O_NONBLOCK,S_IWGRP | S_IWGRP ))<0)//创建日志文件
	{
		SNAI_DEBUG_INFO("log_file open failed");
		perror("LOG: Failed to open the SNAI_log! \n");
		printf("log_file open failed \n");
		return -1;		
	}
	return 1;
}
void SNAI_log_ww(char *str)
{
	if(SNAI_log_id != -1)
	{
		write(SNAI_log_id,str,strlen(str));
	}
	else
	{
		SNAI_log_init();//重启
	}
}

void usart_cleanup(void)
{
	close(SNAI_file_id);//关闭串口1号串口，其实就是关闭文件，通过文件描述符操作descriptor
}
/* 串口发送 */
int usart_tx(unsigned char* msg , int len)
{
	int count = write(SNAI_file_id, msg, len) ;
	if (count < 0)//返回写入的值＜0则错误
	{
		perror("Failed to write to the output.\n");//显示错误信息，到错误流
		SNAI_DEBUG_INFO("Failed to write to the output.");
		return  -1;
	}

	return count ;//正确，则返回写入的字节数
}
/* 串口接收 */
int usart_rx(SNAI_circular_buffer *cb, int len)
{
	int count = read(SNAI_file_id,cb->ptr + cb->write_offset,len);//初次buffer存储地址为cache的分配的起始地址
	if (count < 0)
	{
		SNAI_DEBUG_INFO("Failed to read from the input.");
		perror("Failed to read from the input.\n");
		return -1;
	}
	cb->write_offset += count;//写入的数据统计=读取的数据长度+初始分配地址    是个全局变量 假设发送0-5数据地址=则长度为6个字节 6作为下一次写入地址起始位OK
	return count ;
}
/* 数据丢弃 */
void usart_discard(SNAI_circular_buffer *cb)
{
	cbClear(cb);
}
/* SIGKILL信号处理 */
void usart_sig_hander(int signo)
{
	SNAI_DEBUG_INFO("Usart thread was killed"); 
  printf("Usart thread was killed\n");
	usart_cleanup();
	pthread_exit(0);
}
/*  串口接收的LOOP */
void* usart_rx_start(void* data)
{
	int  fs_sel;
	fd_set fs_read;
	FD_ZERO(&fs_read);
	FD_SET(SNAI_file_id,&fs_read);
	SNAI_circular_buffer *cb = (SNAI_circular_buffer *)(data);
	signal(SIGKILL,usart_sig_hander);

	while(1)
	{
		fs_sel = select(SNAI_file_id+1,&fs_read,NULL,NULL,NULL);
		if(fs_sel)
		{
			if(cb->write_offset > 10240 && cb->read_offset < 8192)//写入大于10K并且,读取小于8K则属于溢出
			{
				SNAI_DEBUG_INFO("Buffer will overflow , discard!");
				printf("Buffer will overflow , discard!\n");//数据溢出
				usart_discard(cb);
				continue;//丢弃一次记录后，退出本次if，若达到条件再次进入
			}
			usart_rx(cb,SNAI_USART_RX_ONE_TIME);//256字节
		}
		else
		{
			SNAI_DEBUG_INFO("Usart receive err .");
			printf("Usart receive err .\n");
			break;
		}
	}
	return NULL;
}
/* SIG_KILL 处理函数 */
void decode_sig_hander(int signo) 
{
	pthread_exit(0);
}
/* TLV解码的LOOP*t,type,l,lenth,v,value*/
void* tlv_decode_start(void* data)
{
    int i;
    SNAI_circular_buffer *cb = (SNAI_circular_buffer *) (data);
    unsigned char msg_type,msg_type2, msg_len,msg_len_water_meter;
    signal(SIGKILL, decode_sig_hander);
    while (1)
    {
        printf("pthread %d is aliver",1);
        
        if (cb_bytes_can_read(cb) >= SNAI_MSG_MIN_LEN) //比较当前可读数据长度=当前写入长度-已读长度）大于8即可进入
        {
            msg_type = *((unsigned char*) (cb->ptr + cb->read_offset)); //读取数据头，含义为消息类型
            msg_type2 = *((unsigned char*) (cb->ptr + cb->read_offset+1)); //读取数据头，含义为消息类型
            msg_len = *((unsigned char*) (cb->ptr + cb->read_offset + 2)); //第二个字节为数据长度
            msg_len_water_meter = (*((unsigned char*) (cb->ptr + cb->read_offset + 2)))+5; //水表情况下，数据长度

//假设读取0-5数据地址=则长度为6个字节 6作为下一次读取地址起始位OK
            if (((msg_type != SNAI_MSG_TYPE_DATA) && (msg_type != 0x11) && (msg_type2 != SNAI_MSG_TYPE_DATA2))
                            || (msg_len < SNAI_MSG_MIN_LEN)) //类型不是0xFF and not is 0xEE/0x11，或者长度小于8。皆属于解码错误
            {
								SNAI_DEBUG_INFO("解码错误！"); 
                printf("pre decode err !\r\n");
                printf("msg_type:%02X msg_len:%u\r\n", msg_type, msg_len);
                cb_read_offset_inc(cb, 1);
            } else { //类型正确的情况下，或者长度正确的情况下
                while (cb_bytes_can_read(cb) < msg_len)
                {
										SNAI_DEBUG_INFO("小于可读等待中......");                       
 										usleep(250);//u秒级挂起线程
                }; //可读的小于消息长度
								
                if(msg_type == 0x11 && msg_len_water_meter == 13)//类型为水表，长度符合
                {
                    SNAI_CRC_value = CRC_Return(cb->ptr + cb->read_offset,msg_len_water_meter-2);
                }
                else
                {
                    SNAI_CRC_value = CRC_Return(cb->ptr + cb->read_offset,msg_len-2);
                }
                SNAI_CRC_value_L =(unsigned char)(SNAI_CRC_value &0x00FF);//有无符号重要！
                SNAI_CRC_value_H = (unsigned char)((SNAI_CRC_value>>8)&0x00FF);
                if((SNAI_CRC_value_L == *((unsigned char*) (cb->ptr + cb->read_offset + msg_len-2))) && (SNAI_CRC_value_H == *((unsigned char*) (cb->ptr + cb->read_offset + msg_len-1))))//校验CRC
                {
                    for (i = 0; i < msg_len; i++)
                    {
                            printf("%02X ",
                                            *((unsigned char*) (cb->ptr + cb->read_offset + i))); //打印2位的16进制数，不足位0补齐。
                            SNAI_all_device_value[i] = *((unsigned char*) (cb->ptr + cb->read_offset + i));//当前数据给数组。
                            SNAI_DEBUG_INFO("接收数据：%02X",SNAI_all_device_value[i]);
                    }
                    printf("\n");
                }
                if((SNAI_CRC_value_L == *((unsigned char*) (cb->ptr + cb->read_offset + msg_len_water_meter-2))) && (SNAI_CRC_value_H == *((unsigned char*) (cb->ptr + cb->read_offset + msg_len_water_meter-1))))//校验CRC
                {
										
										
                    for (i = 0; i < msg_len_water_meter; i++)
                    {
                            printf("%02X ",
                                            *((unsigned char*) (cb->ptr + cb->read_offset + i))); //打印2位的16进制数，不足位0补齐。
                            SNAI_all_device_value[i] = *((unsigned char*) (cb->ptr + cb->read_offset + i));//当前数据给数组。
                            SNAI_DEBUG_INFO("接收水表数据：%02X",SNAI_all_device_value[i]);
                    }
                    printf("\n");
                }
                //sleep(5);//秒级挂起线程
                if ((SNAI_READ_DATA == SNAI_all_device_value[4]) && (msg_len > SNAI_MSG_MIN_LEN))
                {
										pthread_mutex_lock(&SNAI_Decode_mutex_lock);
										//pthread_cond_wait(&SNAI_cond, &SNAI_Decode_mutex_lock);
                    SNAI_DEBUG_INFO("解析传感数据值-->");
                    tlv_decode(cb);
                    cb_read_offset_inc(cb, msg_len);
										//pthread_cond_signal(&SNAI_cond);
										pthread_mutex_unlock(&SNAI_Decode_mutex_lock);
                }
                if ((0x11 == SNAI_all_device_value[0]) && (msg_len_water_meter > SNAI_MSG_MIN_LEN))
                {
										pthread_mutex_lock(&SNAI_Decode_mutex_lock);
										//pthread_cond_wait(&SNAI_cond, &SNAI_Decode_mutex_lock);
                    SNAI_DEBUG_INFO("解析水表传感数据值-->");
                    tlv_decode(cb);
                    cb_read_offset_inc(cb, msg_len_water_meter);
										//pthread_cond_signal(&SNAI_cond);
										pthread_mutex_unlock(&SNAI_Decode_mutex_lock);
                }
                SNAI_DEBUG_INFO("读取地址-->"%lu""%lu"<--写入地址",cb->read_offset,cb->write_offset);
								
            }
        }
				
        usleep(500000);//us级挂起线程
				sleep(1);
    }
    return NULL;
}

/*0XFF 0XEE LEN  485_ADD 0X52/0X51 TYPE1 DATA_LEN H L TYPE2 DATA_LEN2 H L CRC_L CRC_H*/
/* 1     1   1      1        1       1       1    1 1   ?       ?     ? ?   1     1  */
/*Water_Meter*/
/*485ADDR FUNC LEN DATA0 ---DATA7 CRC_L CRC_H*/
void tlv_decode(SNAI_circular_buffer *cb)
{
    float float_temp = 0,sym_bit = 0,float_Humi_temp = 0;
		double flow_rate_value = 0;
    unsigned short Current_Co2_value = 0,Current_NH3_value = 0,Current_light_intensity = 0,Current_Position_L_value = 0,Current_Position_R_value = 0,
            Current_Co_value = 0,Current_Negative_Pressure_value = 0;

    if(SNAI_all_device_value[0] == 0x11 && SNAI_all_device_value[1] == 0x03 && SNAI_all_device_value[2] == 0x08)
    {
				SNAI_DEBUG_INFO("解析【%u】中......",SNAI_all_device_value[0]);
        dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[17]]].type = LEDA_TYPE_DOUBLE;
        strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[17]]].key ,"CurrentWater_Yield");
        sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[17]]].value,"%02X%02X.%02X%02X",SNAI_all_device_value[3],SNAI_all_device_value[4],SNAI_all_device_value[5],SNAI_all_device_value[6]);
        dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[17]]+1].type = LEDA_TYPE_DOUBLE;
        SNAI_DEBUG_INFO("获取17#水量值:【%02X%02X.%02X%02Xm³】瞬时流量:【%02X%02X%02X.%02XL/H】",SNAI_all_device_value[3],SNAI_all_device_value[4],SNAI_all_device_value[5],SNAI_all_device_value[6]
                ,SNAI_all_device_value[7],SNAI_all_device_value[8],SNAI_all_device_value[9],SNAI_all_device_value[10]);
        
        flow_rate_value = (double)((SNAI_all_device_value[7]>>4)*100000+(SNAI_all_device_value[7]&0x0F)*10000+
									(SNAI_all_device_value[8]>>4)*1000+(SNAI_all_device_value[8]&0x0F)*100+
									(SNAI_all_device_value[9]>>4)*10+(SNAI_all_device_value[9]&0x0F)+
									(SNAI_all_device_value[10]>>4)*0.1+(SNAI_all_device_value[10]&0x0F)*0.01);
        flow_rate_value = flow_rate_value/1200.00;//L/H transform L/s
        strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[17]]+1].key ,"CurrentFlow_Rate");
        sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[17]]+1].value,"%.2f",flow_rate_value);
        SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[17]] = 1;
    }
    else//
    {
				SNAI_DEBUG_INFO("解析【%u】中......",SNAI_all_device_value[3]);
        switch (SNAI_all_device_value[5])//先匹配数据类型 再匹配 地址
        {
            case SNAI_TYPE_Boiler:
            case SNAI_TYPE_tmpt:
            case SNAI_TYPE_out_tmpt:
                sym_bit = (SNAI_all_device_value[7] & 0x80?-1.0:1.0);
                unsigned char SNAI_DEVICE_TEMP = (SNAI_all_device_value[7] & 0x7F);//去掉符号位标志
                unsigned short Current_value = SNAI_DEVICE_TEMP*256;
                Current_value += (SNAI_all_device_value[8]>>4)*16+
                                        (SNAI_all_device_value[8]&0x0F);
                float_temp =(float) Current_value;
                float_temp = (sym_bit*(float_temp/10.0));
                if(SNAI_all_device_value[9] == SNAI_TYPE_humi || SNAI_all_device_value[9] == SNAI_TYPE_out_humi)//湿度
                {
                    unsigned char SNAI_DEVICE_TEMP_Humi = (SNAI_all_device_value[11] & 0x7F);//去掉符号位标志
                    unsigned short Current_Humi_value = SNAI_DEVICE_TEMP_Humi*256;
                    Current_Humi_value += (SNAI_all_device_value[12]>>4)*16+
                                            (SNAI_all_device_value[12]&0x0F);
                    float_Humi_temp = (float) Current_Humi_value;
                    float_Humi_temp = (float_Humi_temp/10.0);
                }
                if (sym_bit == -1.0)
                {
                    switch (SNAI_all_device_value[5])
                    {
                    case SNAI_TYPE_Boiler:
                        if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[6] == 1)
                        {
                            dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[6]]].type = LEDA_TYPE_DOUBLE;//LEDA_TYPE_TEXT;//LEDA_TYPE_FLOAT;//浮点
                            strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[6]]].key ,"CurrentTemperature");
                            sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[6]]].value,"%.1f",float_temp);
                            //gcvt(float_temp, 3, dev_proper_data[0].value);
                            SNAI_DEBUG_INFO("获取6#水温数负值【%.1f℃】",float_temp);
                            SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[6]] = 1;
                        }
                        break;
                    case SNAI_TYPE_tmpt:
										case SNAI_TYPE_out_tmpt:
                        if(SNAI_all_device_value[3] == 0x02)//2#WSD
                        {
                            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[2] == 1)
                            {
                                dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[2]]].type = LEDA_TYPE_DOUBLE;//LEDA_TYPE_TEXT;//LEDA_TYPE_FLOAT;//浮点
                                strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[2]]].key ,"CurrentTemperature");
                                sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[2]]].value,"%.1f",float_temp);
                                SNAI_DEBUG_INFO("获取2#温度负值【%.1f℃】",float_temp);
                                SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[2]] = 0;
                            }
                        }
                        if(SNAI_all_device_value[3] == 0x03)//3#WSD
                        {
                            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[3] == 1)
                            {
                                dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[3]]].type = LEDA_TYPE_DOUBLE;
                                strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[3]]].key ,"CurrentTemperature");
                                sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[3]]].value,"%.1f",float_temp);
                                SNAI_DEBUG_INFO("获取3#温度负值【%.1f℃】",float_temp);
                                SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[3]] = 0;
                            }
                        }
                        if(SNAI_all_device_value[3] == 0x04)//4#WSD
                        {
                            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[4] == 1)
                            {
                                dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[4]]].type = LEDA_TYPE_DOUBLE;
                                strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[4]]].key ,"CurrentTemperature");
                                sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[4]]].value,"%.1f",float_temp);
                                SNAI_DEBUG_INFO("获取4#温度负值【%.1f℃】",float_temp);
                                SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[4]] = 0;
                            }
                        }
                        if(SNAI_all_device_value[3] == 0x05)//5#WSD
                        {
                            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[5] == 1)
                            {
                                dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[5]]].type = LEDA_TYPE_DOUBLE;
                                strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[5]]].key ,"CurrentTemperature");
                                sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[5]]].value,"%.1f",float_temp);
                                SNAI_DEBUG_INFO("获取室外5#温度负值【%.1f℃】",float_temp);
                                SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[5]] = 0;
                            }
                        }
                        break;
                    default:
                        break;
                    }
                }
                else//positive num
                {
                    switch (SNAI_all_device_value[5])
                    {
                    case SNAI_TYPE_Boiler:
                        if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[6] == 1)
                        {
                            dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[6]]].type = LEDA_TYPE_DOUBLE;
                            strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[6]]].key ,"CurrentTemperature");
                            sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[6]]].value,"%.1f",float_temp);
                            //gcvt(float_temp, 3, dev_proper_data[0].value);
                            SNAI_DEBUG_INFO("获取6#水温数正值【%.1f】",float_temp);
                            SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[6]] = 1;
                        }
                        break;
                    case SNAI_TYPE_tmpt:
										case SNAI_TYPE_out_tmpt:
                        if(SNAI_all_device_value[3] == 0x02)//2#WSD
                        {
                            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[2] == 1)
                            {
                                dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[2]]].type = LEDA_TYPE_DOUBLE;//LEDA_TYPE_TEXT;//LEDA_TYPE_FLOAT;//浮点
                                strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[2]]].key ,"CurrentTemperature");
                                sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[2]]].value,"%.1f",float_temp);
                                SNAI_DEBUG_INFO("获取2#温度正值【%.1f℃】",float_temp);
                                SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[2]] = 0;
                            }
                        }
                        if(SNAI_all_device_value[3] == 0x03)//3#WSD
                        {
                            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[3] == 1)
                            {
                                dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[3]]].type = LEDA_TYPE_DOUBLE;
                                strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[3]]].key ,"CurrentTemperature");
                                sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[3]]].value,"%.1f",float_temp);
                                SNAI_DEBUG_INFO("获取3#温度正值【%.1f℃】",float_temp);
                                SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[3]] = 0;
                            }
                        }
                        if(SNAI_all_device_value[3] == 0x04)//4#WSD
                        {
                            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[4] == 1)
                            {
                                dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[4]]].type = LEDA_TYPE_DOUBLE;
                                strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[4]]].key ,"CurrentTemperature");
                                sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[4]]].value,"%.1f",float_temp);
                                SNAI_DEBUG_INFO("获取4#温度正值【%.1f℃】",float_temp);
                                SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[4]] = 0;
                            }
                        }
                        if(SNAI_all_device_value[3] == 0x05)//5#WSD
                        {
                            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[5] == 1)
                            {
                                dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[5]]].type = LEDA_TYPE_DOUBLE;
                                strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[5]]].key ,"CurrentTemperature");
                                sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[5]]].value,"%.1f",float_temp);
                                SNAI_DEBUG_INFO("获取室外5#温度正值【%.1f℃】",float_temp);
                                SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[5]] = 0;
                            }
                        }
                        break;
                    default:
                        break;
                    }
                }
                if(SNAI_all_device_value[3] == 0x02 || SNAI_all_device_value[3] == 0x03 || SNAI_all_device_value[3] == 0x04
                        || SNAI_all_device_value[3] == 0x05)
                {
                    switch (SNAI_all_device_value[3])
                    {
                        case 0x02:
                        if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[2] == 1)
                        {
                            dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[2]]+1].type = LEDA_TYPE_DOUBLE;//浮点
                            strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[2]]+1].key ,"CurrentHumidity");
                            sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[2]]+1].value,"%.1f",float_Humi_temp);
                            SNAI_DEBUG_INFO("获取2#湿度值【%.1f%%】",float_Humi_temp);
                            SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[2]] = 1;
                        }
                            break;
                        case 0x03:
                        if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[3] == 1)
                        {
                            dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[3]]+1].type = LEDA_TYPE_DOUBLE;//浮点
                            strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[3]]+1].key ,"CurrentHumidity");
                            sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[3]]+1].value,"%.1f",float_Humi_temp);
                            SNAI_DEBUG_INFO("获取3#湿度值【%.1f%%】",float_Humi_temp);
                            SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[3]] = 1;
                        }
                            break;
                        case 0x04:
                        if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[4] == 1)
                        {
                            dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[4]]+1].type = LEDA_TYPE_DOUBLE;//浮点
                            strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[4]]+1].key ,"CurrentHumidity");
                            sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[4]]+1].value,"%.1f",float_Humi_temp);
                            SNAI_DEBUG_INFO("获取4#湿度值【%.1f%%】",float_Humi_temp);
                            SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[4]] = 1;
                        }
                            break;
                        case 0x05:
                        if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[5] == 1)
                        {
                            dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[5]]+1].type = LEDA_TYPE_DOUBLE;//浮点
                            strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[5]]+1].key ,"CurrentHumidity");
                            sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[5]]+1].value,"%.1f",float_Humi_temp);
                            SNAI_DEBUG_INFO("获取室外5#湿度值【%.1f%%】",float_Humi_temp);
                            SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[5]] = 1;
                        }
                            break;
                        default:
                           break;
                    }
                }
                    break;
        case SNAI_TYPE_Co2:
            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[8] == 1)
            {
                Current_Co2_value = (SNAI_all_device_value[7] & 0x7F)*256;
                Current_Co2_value += (SNAI_all_device_value[8]>>4)*16+
                                        (SNAI_all_device_value[8]&0x0F);
                dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[8]]].type = LEDA_TYPE_INT;
                strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[8]]].key ,"GasConcentration");
                sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[8]]].value,"%u",Current_Co2_value);
                SNAI_DEBUG_INFO("获取8#二氧化碳值【%uppm】",Current_Co2_value);
                SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[8]] = 1;
            }
            break;
        case SNAI_TYPE_NH3:
            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[9] == 1)
            {
                Current_NH3_value = (SNAI_all_device_value[7] & 0x7F)*256;
                Current_NH3_value += (SNAI_all_device_value[8]>>4)*16+
                                        (SNAI_all_device_value[8]&0x0F);
                dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[9]]].type = LEDA_TYPE_INT;
                strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[9]]].key ,"GasConcentration");
                sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[9]]].value,"%u",Current_NH3_value);
                SNAI_DEBUG_INFO("获取9#氨气值【%uppm】",Current_NH3_value);
                SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[9]] = 1;
            }
            break;
        case SNAI_TYPE_Illumination:
            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[10] == 1)
            {
                Current_light_intensity = (SNAI_all_device_value[7] & 0x7F)*256;
                Current_light_intensity += (SNAI_all_device_value[8]>>4)*16+
                                        (SNAI_all_device_value[8]&0x0F);
                dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[10]]].type = LEDA_TYPE_INT;
                strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[10]]].key ,"Light_illumination");
                sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[10]]].value,"%u",Current_light_intensity);
                SNAI_DEBUG_INFO("获取10#光照值【%uLux】",Current_light_intensity);
                SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[10]] = 1;
            }
            break;
        case SNAI_TYPE_Position_Left:
            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[11] == 1)
            {
                Current_Position_L_value = (SNAI_all_device_value[7] & 0x7F)*256;
                Current_Position_L_value += (SNAI_all_device_value[8]>>4)*16+
                                        (SNAI_all_device_value[8]&0x0F);
                dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[11]]].type = LEDA_TYPE_INT;
                strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[11]]].key ,"Position_Value");
                sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[11]]].value,"%u",Current_Position_L_value);
                SNAI_DEBUG_INFO("获取11#位置LEFT传感器偏移距离【%umm】",Current_Position_L_value);
                SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[11]] = 1;
            }
            break;
        case SNAI_TYPE_Position_Right:
            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[12] == 1)
            {
                Current_Position_R_value = (SNAI_all_device_value[7] & 0x7F)*256;
                Current_Position_R_value += (SNAI_all_device_value[8]>>4)*16+
                                        (SNAI_all_device_value[8]&0x0F);
                dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[12]]].type = LEDA_TYPE_INT;
                strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[12]]].key ,"Position_Value");
                sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[12]]].value,"%u",Current_Position_R_value);
                SNAI_DEBUG_INFO("获取12#位置RIGHT传感器偏移距离【%umm】",Current_Position_R_value);
                SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[12]] = 1;
            }
            break;
        case SNAI_TYPE_Co:
            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[13] == 1)
            {
                Current_Co_value = (SNAI_all_device_value[7] & 0x7F)*256;
                Current_Co_value += (SNAI_all_device_value[8]>>4)*16+
                                        (SNAI_all_device_value[8]&0x0F);
                dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[13]]].type = LEDA_TYPE_INT;
                strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[13]]].key ,"GasConcentration");
                sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[13]]].value,"%u",Current_Co_value);
                SNAI_DEBUG_INFO("获取13#一氧化碳值【%uppm】",Current_Co_value);
                SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[13]] = 1;
            }
            break;

        case SNAI_TYPE_Water_Meter:
                //this is not standrad agreement
            break;
        case SNAI_TYPE_Negative_Pressure:
            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[7] == 1)
            {
                Current_Negative_Pressure_value = (SNAI_all_device_value[7] & 0x7F)*256;
                Current_Negative_Pressure_value += (SNAI_all_device_value[8]>>4)*16+
                                        (SNAI_all_device_value[8]&0x0F);
                dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[7]]].type = LEDA_TYPE_INT;
                strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[7]]].key ,"Current_Negative_pressure");
                sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[7]]].value,"%u",Current_Negative_Pressure_value);
                SNAI_DEBUG_INFO("获取7#负压值【%uPa】",Current_Negative_Pressure_value);
                SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[7]] = 1;
            }
            break;
        case SNAI_TYPE_Wind_Direction:
            break;
        case SNAI_TYPE_Wind_Speed:
            break;

        case SNAI_TYPE_Other:
            break;
        default:
            break;
        }
    }
}
void* status_report(void* data)
{
        int i = 0;
        int Para_num = 0,temp = 0;
        SNAI_circular_buffer *cb = (SNAI_circular_buffer *)(data);
        signal(SIGKILL,SNAI_driver_exit);
         /* 对已上线设备每隔5秒钟上报一次温度数据和事件 */
    while (1)
    {
				pthread_mutex_trylock(&SNAI_Decode_mutex_lock);
        for (i = 0; i < g_dev_handle_count; i++)
        {
						
            if(SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[i] == 1)
            {
		          /* report device properties */
              temp = SNAI_ALL_DEVICE_REPORT.Parameter_ptr[i]+SNAI_ALL_DEVICE_REPORT.Parameter_count[i];
							SNAI_DEBUG_INFO("最大偏移至数组下标【%d】",temp);
              for(Para_num = SNAI_ALL_DEVICE_REPORT.Parameter_ptr[i];Para_num < temp;Para_num++)
		          {
									if(0 < strlen(dev_proper_data[Para_num].value))
									{
				            SNAI_DEBUG_INFO("上传时设备数值【%s】",dev_proper_data[Para_num].value);
				            SNAI_DEBUG_INFO("上传时设备字符数【%lu】",strlen(dev_proper_data[Para_num].value));
									}
              }
							SNAI_DEBUG_INFO("上传数据首地址:【%d】数量:【%d】",SNAI_ALL_DEVICE_REPORT.Parameter_ptr[i],SNAI_ALL_DEVICE_REPORT.Parameter_count[i]);
              leda_report_properties(g_dev_handle_list[i],dev_proper_data+SNAI_ALL_DEVICE_REPORT.Parameter_ptr[i], SNAI_ALL_DEVICE_REPORT.Parameter_count[i]);
                SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[i] = 0;
								//memset(dev_proper_data+SNAI_ALL_DEVICE_REPORT.Parameter_ptr[i],0,sizeof(dev_proper_data[0])*SNAI_ALL_DEVICE_REPORT.Parameter_count[i]);//不可瞎用，导致乱码！
            }
            else
            {
                SNAI_DEBUG_INFO("设备未准备好数据，不上传！【线程号“%d”】",i);
            }
            /* report device event */
            /*if(!strcmp(dev_event_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[i]],))
            dev_event_data[1] =

                    .type  = LEDA_TYPE_ENUM,
                    .key   = {"Faultreport"},
                    .value = {"0"}//无故障//1传感器故障

            leda_report_event(g_dev_handle_list[i], "Faultreport", dev_event_data, 1);*/
						
        }
				pthread_mutex_unlock(&SNAI_Decode_mutex_lock);
        sleep(2);
    }
}

void* TX_READ(void* data)
{
    SNAI_circular_buffer *cb = (SNAI_circular_buffer *) (data);
    unsigned char SNAI_DEVICE_ReadBUFF[9] = {0xFF,0xEE,0x08,0x02,0x52,0x01,0xE2,0xC2};
    static unsigned short times = 1;
    unsigned short SNAI_TX_CRC_value = 0;
    unsigned char SNAI_TX_CRC_value_L = 0;
    unsigned char SNAI_TX_CRC_value_H = 0;
    unsigned char SNAI_485_ADDR = 0x02;//start_origin
    while(1)
    {
        if(SNAI_file_id == -1)
        {
            usart_init();
            SNAI_DEBUG_INFO("再次尝试打开！");
        }
        else
        {	
            SNAI_DEBUG_INFO("串口发送%u次",times);
            times ++;
            times = (times >= 30000?1:times);
            SNAI_DEVICE_ReadBUFF[3] = SNAI_485_ADDR;
            if(SNAI_485_ADDR == 0x11)
            {
                SNAI_DEVICE_ReadBUFF[0] = 0x11;
                SNAI_DEVICE_ReadBUFF[1] = 0x03;
                SNAI_DEVICE_ReadBUFF[2] = 0x00;
                SNAI_DEVICE_ReadBUFF[3] = 0x00;
                SNAI_DEVICE_ReadBUFF[4] = 0x00;
                SNAI_DEVICE_ReadBUFF[5] = 0x04;
            }
            else
            {
                SNAI_DEVICE_ReadBUFF[0] = 0xFF;
                SNAI_DEVICE_ReadBUFF[1] = 0xEE;
                SNAI_DEVICE_ReadBUFF[2] = 0x08;
								SNAI_DEVICE_ReadBUFF[4] = 0x52;
								SNAI_DEVICE_ReadBUFF[5] = 0x01;
            }
            SNAI_485_ADDR++;
            SNAI_485_ADDR = (SNAI_485_ADDR > 20?0x02:SNAI_485_ADDR);
            SNAI_TX_CRC_value = CRC_Return(SNAI_DEVICE_ReadBUFF,6);
            SNAI_TX_CRC_value_L =(unsigned char)(SNAI_TX_CRC_value &0x00FF);//有无符号重要！
            SNAI_TX_CRC_value_H = (unsigned char)((SNAI_TX_CRC_value>>8)&0x00FF);
            SNAI_DEVICE_ReadBUFF[6] = SNAI_TX_CRC_value_L;
            SNAI_DEVICE_ReadBUFF[7] = SNAI_TX_CRC_value_H;
            usart_tx(SNAI_DEVICE_ReadBUFF ,8);
						for(int TX_data = 0;TX_data < 8;TX_data++)
						{
								SNAI_DEBUG_INFO("发送数据:%02X",SNAI_DEVICE_ReadBUFF[TX_data]);
						}
//          if(cb_bytes_can_read(cb) < SNAI_MSG_MIN_LEN)
						
        }
				usleep(1000);//1ms
        sleep(1);   
    }
}

static unsigned short Crc_Cal(unsigned short Data, unsigned short GenPoly, unsigned short CrcData) 
{
	unsigned short TmpI;
	Data *= 2;
	for (TmpI = 8; TmpI > 0; TmpI--) {
		Data = Data / 2;
		if ((Data ^ CrcData) & 1)
			CrcData = (CrcData / 2) ^ GenPoly;
		else
			CrcData /= 2;
	}
	return CrcData;
}
unsigned short CRC_Return(unsigned char *Crc_Buf, unsigned char Crc_Len)
{
        unsigned short temp;
        unsigned short CRC_R = 0xFFFF;
        for (temp = 0; temp < Crc_Len; temp++)
        {
                CRC_R = Crc_Cal(Crc_Buf[temp], 0xA001, CRC_R);
        }
        return CRC_R;
}


/////////////////////////////////////////////////////////////////////////

/*
*演示：获取设备属性
*/
static int get_properties_callback_cb(device_handle_t dev_handle, 
                               leda_device_data_t properties[], 
                               int properties_count, 
                               void *usr_data)
{
    int i = 0;
    for (i = 0; i < properties_count; i++)
    {
        log_i(LED_TAG_NAME, "get_property %s: ", properties[i].key);
        SNAI_DEBUG_INFO("获取设备属性来自云端【%s】",properties[i].key);
        if (!strcmp(properties[i].key, "temperature"))//比较 属性或事件名 字符串是否一致，返回值为0则一致
        {
            /* 作为演示，填写获取属性数据为模拟数据 */
            properties[i].type = LEDA_TYPE_INT;//整形数据integer
            sprintf(properties[i].value, "%d", 30);

            log_i(LED_TAG_NAME, "%s\r\n",  properties[i].value);
        }
    }

    return LE_SUCCESS;
}
/*
*演示：设置设备属性
*/
static int set_properties_callback_cb(device_handle_t dev_handle, 
                               const leda_device_data_t properties[], 
                               int properties_count, 
                               void *usr_data)
{
    int i = 0;
    for (i = 0; i < properties_count; i++)
    {
        /* 作为演示，仅打印出设置属性信息 */
        log_i(LED_TAG_NAME, "set_property type:%d %s: %s\r\n", properties[i].type, properties[i].key, properties[i].value);
        SNAI_DEBUG_INFO("设置设备属性来自云端【%u】【%s】【%s】",properties[i].type,properties[i].key, properties[i].value);
    }

    return LE_SUCCESS;
}
/*
*演示：调用设备服务，并反馈
*/
static int call_service_callback_cb(device_handle_t dev_handle, 
                               const char *service_name, 
                               const leda_device_data_t data[], 
                               int data_count, 
                               leda_device_data_t output_data[], 
                               void *usr_data)
{
    int i = 0;

    /* service_name为该驱动物模型自定义方法名称 */
    log_i(LED_TAG_NAME, "service_name: %s\r\n", service_name);
    SNAI_DEBUG_INFO("service_name:【%s】",service_name);
    /* 获取service_name方法的参数名称和值信息 */
    for (i = 0; i < data_count; i++)
    {
        log_i(LED_TAG_NAME, "input_data %s: %s\r\n", data[i].key, data[i].value);
        SNAI_DEBUG_INFO("input_data:【%s】【%s】",data[i].key,data[i].value);
    }

    /* 此处错位演示并没有执行真正的自定义方法 */

    return LE_SUCCESS;
}

static int get_and_parse_deviceconfig(const char* module_name)
{
    int                         ret             = LE_SUCCESS;

    int                         size            = 0;
    char                        *device_config  = NULL;

    char                        *productKey     = NULL;
    char                        *deviceName     = NULL;

    cJSON                       *root           = NULL;
    cJSON                       *object         = NULL;
    cJSON                       *item           = NULL;
    cJSON                       *result         = NULL;
		char 												*SNAI_DEVICE_NAME_ANALYSIS = NULL;
    leda_device_callback_t      device_cb;//定义一个结构体，其成员为回调函数指针（作用为获取设备属性，设置属性，设备服务，服务回调结果数组最大长度设置
    device_handle_t             dev_handle      = -1;

    /* 获取驱动设备配置 */
    size = leda_get_config_size(module_name);//得到长度
    if (size >0)
    {
        device_config = (char*)malloc(size);
        if (NULL == device_config)
        {
            log_e(LED_TAG_NAME, "allocate memory failed\r\n");//名称标签，字符格式，参数...--》等级，颜色，名称标签，文件名，函数名，行号，字符格式，参数...
			SNAI_DEBUG_INFO("设备配置内存分配失败！！");
            return LE_ERROR_INVAILD_PARAM;
        }

        if (LE_SUCCESS != (ret = leda_get_config(module_name, device_config, size)))//得到另一进程数据(device_config)并匹配上次获取的长度是否一致
        {
			SNAI_DEBUG_INFO("获取驱动设备配置失败！！");
            log_e(LED_TAG_NAME, "get device config failed\r\n");
            return ret;
        }
    }

    /* 解析驱动设备配置 */
    root = cJSON_Parse(device_config);
    if (NULL == root)
    {
		SNAI_DEBUG_INFO("解析驱动设备配置失败！！");
        log_e(LED_TAG_NAME, "device config parser failed\r\n");
        return LE_ERROR_INVAILD_PARAM;
    }

    object = cJSON_GetObjectItem(root, "deviceList");//解析json数据
    cJSON_ArrayForEach(item, object)
    {
        if (cJSON_Object == item->type)
        {
            /* 按照配置格式解析内容 */
            result = cJSON_GetObjectItem(item, "productKey");
            productKey = result->valuestring;//获取产品标识
			SNAI_DEBUG_INFO("获取产品标识【%s】",productKey);
           
            result = cJSON_GetObjectItem(item, "deviceName");
            deviceName = result->valuestring;//获取设备名称
			SNAI_DEBUG_INFO("获取设备标识【%s】",deviceName);
          
            /* TODO: 解析设备自定义配置信息custom，该字段内容来源在云端控制台的驱动配置项。由于该字段内容
               为字符串的json内容，所以在去除custom的value值后，需要再次进行json解析操作。
            */

            /* 注册并上线设备 */
            device_cb.get_properties_cb            = get_properties_callback_cb;//赋予实际回调函数
            device_cb.set_properties_cb            = set_properties_callback_cb;
            device_cb.call_service_cb              = call_service_callback_cb;
            device_cb.service_output_max_count     = 5;

            dev_handle = leda_register_and_online_by_device_name(productKey, deviceName, &device_cb, NULL);//调用回调获取属性后，设备注册上线到阿里云物联网平台
            if (dev_handle < 0)
            {
                log_e(LED_TAG_NAME, "product:%s device:%s register failed\r\n", productKey, deviceName);
                continue;
            }
            SNAI_DEVICE_NAME_ANALYSIS = (char*)malloc(sizeof(char)*5);
            int analysis_ret = 0;
            if(NULL == SNAI_DEVICE_NAME_ANALYSIS)
            {
                SNAI_DEBUG_INFO("设备名称解析内存分配失败！");
                return LE_ERROR_INVAILD_PARAM;
            }
            analysis_ret = sscanf(deviceName,"%*[^:]:%s",SNAI_DEVICE_NAME_ANALYSIS);
            if(analysis_ret != 0)
            {
                SNAI_DEBUG_INFO("解析匹配个数:【%d】解析地址为:【%s】",analysis_ret,SNAI_DEVICE_NAME_ANALYSIS);
                SNAI_DEVICE_RS485_ADDR_HANDLE(dev_handle,SNAI_DEVICE_NAME_ANALYSIS);
            }
						free(SNAI_DEVICE_NAME_ANALYSIS);
						SNAI_DEVICE_NAME_ANALYSIS = NULL;
            g_dev_handle_list[g_dev_handle_count++] = dev_handle;
            log_i(LED_TAG_NAME, "product:%s device:%s register success\r\n", productKey, deviceName);
            SNAI_DEBUG_INFO("注册进程【%d】",dev_handle);
            SNAI_DEBUG_INFO("设备在线【%d】",g_dev_handle_count);
        }
    }
    if (NULL != root)
    {
        cJSON_Delete(root);
    }

    free(device_config);

    return LE_SUCCESS;
}
/*Preset 每个设备线程号&数据存储首地址*/
void SNAI_DEVICE_RS485_ADDR_HANDLE(device_handle_t SNAI_handle,char *RS485_ADDR)
{
    const char *dat[]={
"0000",
"0001",
"0002",
"0003",
"0004",
"0005",
"0006",
"0007",
"0008",
"0009",
"0010",
"0011",
"0012",
"0013",
"0014",
"0015",
"0016",
"0017",
"0018",
"0019",
"0020",
"0021",
"0022",
"0023",
"0024"
    };
    static int DEVICE_PAR_Ptr = 0;
    SNAI_DEBUG_INFO("本次上报参数存入首地址【dev_proper_data[%d]】",DEVICE_PAR_Ptr);
    for(int i =0;i<25;i++)
    {
        if (!strcmp(RS485_ADDR, dat[i]))
        {
            SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[i] = 1;
            SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[i] = SNAI_handle;//解析数据读取此处线程号，作为ptr数据存储下标
						SNAI_DEBUG_INFO("当前线程号作为下标【%d】",SNAI_handle);
            SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_handle] = DEVICE_PAR_Ptr;//线程号对应上报数据 存放地址
            if(i == 2 || i == 3 || i == 4 || i == 5  || i == 17)
            {
                SNAI_ALL_DEVICE_REPORT.Parameter_count[SNAI_handle] = 2;//参数个数
                DEVICE_PAR_Ptr += 2;//下次存放地址offset
            }
            else
            {
                SNAI_ALL_DEVICE_REPORT.Parameter_count[SNAI_handle] = 1;//参数个数
                DEVICE_PAR_Ptr += 1;//下次存放地址offset

            }
            break;
        }
    }
    SNAI_DEBUG_INFO("下次上报参数存入首地址【dev_proper_data[%d]】",DEVICE_PAR_Ptr);
}
int main(int argc, char** argv)
{
		int    ret         = LE_SUCCESS;
    char*  module_name = NULL;
		SNAI_log_init();
    log_init(LED_TAG_NAME, LOG_STDOUT, LOG_LEVEL_DEBUG, LOG_MOD_BRIEF);
		SNAI_DEBUG_INFO("demo startup！");
    log_i(LED_TAG_NAME, "demo startup\r\n");
		for(int snaist = 0;snaist < 25;snaist++)
		{
			SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[snaist] = 0;	
		}
    /* 初始驱动 */
    module_name = leda_get_module_name();//获取设备接入驱动名称（该名称为在物联网平台控制台上传驱动时填写的驱动名称）,读取环境变量参数值
    if (NULL == module_name)
    {
				SNAI_DEBUG_INFO("模块名称未知退出！！");
        log_e(LED_TAG_NAME, "the driver no deploy or deploy failed\r\n");
        return LE_ERROR_UNKNOWN;
    }
		SNAI_DEBUG_INFO("模块名称【%s】",module_name);
  
    if (LE_SUCCESS != (ret = leda_init(module_name, 5)))//接口完成资源初始化
    {
		SNAI_DEBUG_INFO("模块初始化失败退出！！\n");
        log_e(LED_TAG_NAME, "leda_init failed\r\n");
        return ret;
    }

    /* 解析配置 */
    if (LE_SUCCESS != (ret = get_and_parse_deviceconfig(module_name)))
    {
				SNAI_DEBUG_INFO("解析配置失败！！");
        log_e(LED_TAG_NAME, "parse device config failed\r\n");
        return ret;
    }
	/////////////////////////////////////////////////////////////////////////	
		
	/* 8K rx cache for usart rx*/
  SNAI_circular_buffer* cb_usart_rx = cb_create(13);//建立环形缓冲区
	SNAI_pthread_opt_user pthread_user_seq[] =//声明.结构体数组，4个元素，每个元素依次：函数指针、void型指针变量、线程id
	{
		{TX_READ, cb_usart_rx},//元素1，数组【0】，因每个元素是结构体所以具有大括号{}
		{usart_rx_start, cb_usart_rx},
		//{status_minitor,NULL},//数组【2】狀態檢測
		{tlv_decode_start,cb_usart_rx},//数组【2】解碼
		{status_report,cb_usart_rx},//设备数据上报
		{NULL ,NULL}//数组【3】
	};
        
	opt_seq_ptr = pthread_user_seq;//opt_seq_ptr指向pthread_user_seq，函数队列usart_rx_start、tlv_decode_start、status_minitor
/*
 * 			SIGINT   2    采用ctrl+c产生该信号
        SIGQUIT  3    采用ctrl+\产生该信号
        SIGKILL  9    采用kill -9产生该信号
 *
 * */
	signal(SIGINT , main_thread_hander);//只能结束前台进程
	signal(SIGKILL, main_thread_hander);//强制结束
	signal(SIGTERM, main_thread_hander);//可以被阻塞、处理和忽略
	usart_init();
	
  SNAI_DEBUG_INFO("初始化完成！");
	pthread_opt_seq_exec(pthread_user_seq);

	printf("clean up!\r\n");
	/////////////////////////////////////////////////////////////////////////  

  leda_exit(); 
	return LE_SUCCESS;
}

#ifdef __cplusplus
}
#endif
