﻿/*
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
#include <ctype.h>
#include <sqlite3.h>
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
#define ALL_DEVICE_COUNT        24
#define	ALL_DEVICE_Handle		24
#define SNAI_Filtration_Timeout 30
#define MAX_VALUE  64

leda_device_data_t dev_proper_data[ALL_DEVICE_COUNT*2+2] = {
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
#define PRINT_ERRMSG(STR) fprintf(stderr,"line:%d,msg:%s,eMsg:%s\n", __LINE__, STR, strerror(errno))
#define SNAI_DEBUG_INFO(fmt, args...) if(SNAI_TEMP_log != NULL)									\
									{															\
										SNAI_cur_time = time(NULL);								\
										SNAI_get_timestamp(SNAI_buf_date, 20, SNAI_cur_time);	\
										sprintf(SNAI_TEMP_log,									\
										"LOG_%s-FILE:%s-%s(%d)--INFOR:"#fmt"\n", 				\
										SNAI_buf_date,__FILE__,__FUNCTION__,__LINE__,##args);	\
										SNAI_log_ww(SNAI_TEMP_log);								\
									}
leda_device_data_t dev_event_data[ALL_DEVICE_COUNT*5+1] =  //每个设备事件参数总计最多5个
{
				{
					.type  = LEDA_TYPE_ENUM,
					.key   = {"error"},
					.value = {"0"}//无故障//1传感器故障
				}
};
///////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct _option {
  char    key[MAX_VALUE];   /* 对应键 */
  char    value[MAX_VALUE]; /* 对应值 */
  struct  _option *next;    /* 链表连接标识 */
}SNAI_Option;
 
typedef struct _data {
  char    section[MAX_VALUE]; /* 保存section值          */
  SNAI_Option  *option;       /* option链表头           */
  struct  _data *next;        /* 链表连接标识           */
}SNAI_Data;
 
typedef struct {
  char    comment;              /* 表示注释的符号    */
  char    separator;            /* 表示分隔符        */
  char    re_string[MAX_VALUE]; /* 返回值字符串的值  */
  int     re_int;               /* 返回int的值       */
  bool    re_bool;              /* 返回bool的值      */
  double  re_double ;           /* 返回double类型    */
  SNAI_Data    *data;           /* 保存数据的头      */
}SNAI_Config;
void SNAI_delete_config(SNAI_Config *cnf);
bool SNAI_str_empty(const char *string);
bool SNAI_cnf_write_file(SNAI_Config *cnf, const char *filename, const char *header);
bool SNAI_cnf_add_option(SNAI_Config *cnf, const char *section, const char *key, const char *value);
bool SNAI_strip_comments(char *string, char comment);
SNAI_Config *SNAI_cnf_read_config(const char *filename, char comment, char separator);
bool SNAI_cnf_get_value(SNAI_Config *cnf, const char *section, const char *key);
SNAI_Data *SNAI_cnf_has_section(SNAI_Config *cnf, const char *section);
bool SNAI_cnf_remove_option(SNAI_Config *cnf, const char *section, const char *key);
SNAI_Option *SNAI_cnf_has_option(SNAI_Config *cnf, const char *section, const char *key);
bool SNAI_cnf_remove_section(SNAI_Config *cnf, const char *section);
void SNAI_print_config(SNAI_Config *cnf);
///////////////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct SNAI_ALL_DEVICE_EVENT
{
	char SNAI_DEVICE_EVENT_NAME[ALL_DEVICE_COUNT*3+1][20];//每个设备事件最多3个
	int  Event_Parameter_ptr[ALL_DEVICE_Handle+1];//句柄号区分
	int  Event_Parameter_count[ALL_DEVICE_Handle+1];
	char SNAI_DEVICE_EVENT_TRIGGER[ALL_DEVICE_Handle+1];
	int  Event_Name_ptr[ALL_DEVICE_Handle+1];//句柄号区分,存放事件名
	int  Event_Name_count[ALL_DEVICE_Handle+1];
}SNAI_ALL_DEVICE_EVENT_t;
SNAI_ALL_DEVICE_EVENT_t SNAI_DEVICE_EVN_REPOR =
{
		.SNAI_DEVICE_EVENT_NAME = {{0},{0}},
		.Event_Parameter_ptr	= {0},
		.Event_Parameter_count	= {0},
		.SNAI_DEVICE_EVENT_TRIGGER = {0},
		.Event_Name_ptr         = {0},
		.Event_Name_count		= {0}
};

/*
SNAI_ALL_DEVICE_EVENT_t SNAI_DEVICE_EVE_REPOR = 
{
	.dev_event_data_fault ={
			{
	            .type  = LEDA_TYPE_ENUM,
	            .key   = {"error"},
	            .value = {"0"}//无故障//1传感器故障
	        }
		},
	.dev_event_data_alarm1 ={
			{
	            .type  = LEDA_TYPE_ENUM,
	            .key   = {"error"},
	            .value = {"0"}//无故障//1传感器故障
	        }
		},
	.dev_event_data_alarm2 ={
			{
	            .type  = LEDA_TYPE_ENUM,
	            .key   = {"error"},
	            .value = {"0"}//无故障//1传感器故障
	        }
		},
	.SNAI_DEVICE_EVENT_NAME ={0}
};*/

typedef struct SNAI_RS485_HANDLE_NUM
{
        device_handle_t  SNAI_485dev_handle[ALL_DEVICE_COUNT+1];//485地址区分
        int              Parameter_count[ALL_DEVICE_Handle+1];//句柄号区分
        int              Parameter_ptr[ALL_DEVICE_Handle+1];//句柄号区分
        bool             SNAI_device_ready[ALL_DEVICE_Handle+1];////句柄号区分
        bool             SNAI_DEVICE_EXIST[ALL_DEVICE_COUNT+1];//485地址区分
        unsigned char	 SNAI_485dev_ADDR[ALL_DEVICE_Handle+1];//句柄号区分
        unsigned char	 SNAI_485dev_Data_Filtration_Switch[ALL_DEVICE_COUNT+1];//过滤器开启开关
		unsigned char 	 SNAI_485dev_Data_Filtration_Flag[ALL_DEVICE_COUNT+1];//首次进入区分过滤
		int              SNAI_485dev_Data_Filtration_Date_Origin_M[ALL_DEVICE_COUNT+1];//原始时间点记录
		int              SNAI_485dev_Data_Filtration_Date_Origin_H[ALL_DEVICE_COUNT+1];//原始时间点记录
}SNAI_RS485_HANDLE_NUM_t;
SNAI_RS485_HANDLE_NUM_t SNAI_ALL_DEVICE_REPORT;
typedef struct SNAI_RS485_Alarm
{
		float  SNAI_485dev_Alarm_DATA_TMP[ALL_DEVICE_COUNT+1];
		float  SNAI_485dev_Alarm_DATA_Humi[ALL_DEVICE_COUNT+1];
		unsigned int SNAI_485dev_Alarm_DATA_INT[ALL_DEVICE_COUNT+1];//485地址区分 -每个地址报警值
		double SNAI_485dev_Alarm_DATA_Water;
}SNAI_RS485_Alarm_t;
SNAI_RS485_Alarm_t SNAI_Alarm_value = 
{
		.SNAI_485dev_Alarm_DATA_TMP 	= {0},
		.SNAI_485dev_Alarm_DATA_Humi 	= {0},
		.SNAI_485dev_Alarm_DATA_Water 	= 0,
		.SNAI_485dev_Alarm_DATA_INT 	= {0}
};
typedef struct SNAI_RS485_Period
{
		unsigned int  SNAI_485dev_Polling_Period[ALL_DEVICE_COUNT+1];//485地址区分 -对每个地址获取数据周期 /ms*1000
		int  		  SNAI_485dev_Filtration_Timeout_Period[ALL_DEVICE_COUNT+1];//485地址区分 -对每个设备最大过滤数据周期 /min
		unsigned int  SNAI_Gateway_Report_Period;//网关上报数据周期控制 /s
		int 		  SNAI_Main_Filtration_Flag;//总上传启用标志 /0 or 1
		unsigned	short SNAI_polling_cycle[ALL_DEVICE_COUNT+1];
}SNAI_RS485_Period_t;
SNAI_RS485_Period_t SNAI_Period_Set = 
{
		.SNAI_485dev_Polling_Period				 = {0},//500ms
		.SNAI_485dev_Filtration_Timeout_Period	 = {0},
		.SNAI_Gateway_Report_Period 			 = 2,
		.SNAI_Main_Filtration_Flag 				 = 1,
		.SNAI_polling_cycle 					 = {0}
};
volatile int SNAI_Gateway_Power_Switch_Flag = 1;

/*
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
*/
typedef struct SNAI_DEVICE_OLD_DATA
{
        float  SNAI_485dev_OLD_DATA_TMP[7];
		float  SNAI_485dev_OLD_DATA_Humi[6];
		unsigned short SNAI_485dev_OLD_DATA_INT[14];
		/*unsigned short SNAI_485dev_OLD_DATA_FY_7;
		unsigned short SNAI_485dev_OLD_DATA_Co2_8;
		unsigned short SNAI_485dev_OLD_DATA_NH3_9;
		unsigned short SNAI_485dev_OLD_DATA_LIGHT_10;
		unsigned short SNAI_485dev_OLD_DATA_Position_L_11;
		unsigned short SNAI_485dev_OLD_DATA_Position_R_12;
		unsigned short SNAI_485dev_OLD_DATA_Co_13;*/
		double SNAI_485dev_OLD_DATA_Flow_Rate_17;
		double SNAI_485dev_OLD_DATA_Accumulate_17;  
}SNAI_DEVICE_OLD_DATA_t;
SNAI_DEVICE_OLD_DATA_t SNAI_ALL_DEVICE_OLD_DATA =
{
     .SNAI_485dev_OLD_DATA_TMP 				= {0},
     .SNAI_485dev_OLD_DATA_Humi	 			= {0},
     .SNAI_485dev_OLD_DATA_INT 				= {0},
     .SNAI_485dev_OLD_DATA_Flow_Rate_17 	= 0,
     .SNAI_485dev_OLD_DATA_Accumulate_17 	= 0
};
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
#define SNAI_TYPE_Position     		0xAD/*11*/
#define SNAI_TYPE_Other             0xAF

char SNAI_TEMP_log[2048] = {0};		
unsigned short SNAI_CRC_value = 0;//2个字节长度
static int SNAI_file_id = -1;
static int SNAI_log_id = -1;
char SNAI_LOG_File_TIME[40] = {0};
pthread_mutex_t SNAI_Decode_mutex_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t SNAI_GET_Properties_mutex_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t SNAI_cond = PTHREAD_COND_INITIALIZER;

unsigned char SNAI_all_device_value[15]={0};//串口接收数据写入

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
bool SNAI_RS485_DATA_Filtration(unsigned char RS485_ADDR,void *data,unsigned char Parameter_n,unsigned char type); 
bool Check_Filtration_Timeout(unsigned char addr);
unsigned char SNAI_hexCharToValue(unsigned char ch);
//void cb_read_offset_sync(SNAI_circular_buffer *cb);
unsigned char SNAI_hexCharToValue(unsigned char ch)
{
	unsigned char result = 0;
	//获取16进制的高字节位数据
	if (ch >= '0' && ch <= '9')
	{
		result = ch - '0';
	}
	else if (ch >= 'a' && ch <= 'z')
	{
		result = ch - 'a' + 10;
	}
	else if (ch >= 'A' && ch <= 'Z')
	{
		result = ch - 'A' + 10;
	}
	else
	{
		result = 0;
	}
	return result;
}
//--------------------------database_SQL----------------------------------------
typedef  void (*pfun)(char** pr , int row , int column );
sqlite3* db = NULL;
const char* file_database_path = "/linkedge/run/SNAI.db";
const char* sql_query_data = "SELECT count(*) FROM sqlite_master device_info WHERE type='table' AND name='device_info';";
const char* sql_empty_data = "delete from %s;";
const char* sql_create_data = "CREATE TABLE IF NOT EXISTS device_info(id INTEGER , %s INTEGER PRIMARY KEY, device_handle INTEGER, update_time UNSIGNED,period INTEGER,value INTEGER,value1 INTEGER,state INTEGER);";
const char* sql_insert_data = "INSERT OR REPLACE INTO MAIN.device_info VALUES('%d', '%d', '%d', '%ld','%d','%d','%d',%d);";
const char* sql_delete_data = "DELETE FROM MAIN.device_info WHERE addr = %d;";
const char* sql_update_data_time_field = "UPDATE MAIN.device_info SET update_time = %ld where addr = %d;";
const char* sql_search_data = "SELECT * FROM MAIN.device_info WHERE update_time <= %ld-(period*1);";
const char* sql_search_net = "SELECT * FROM MAIN.device_info WHERE addr = %d;";
const char* sql_update_data = "UPDATE MAIN.device_info SET period = %d,value = %d WHERE addr = %d;";
const char* sql_update_state = "UPDATE MAIN.device_info SET state = %d WHERE addr = %d;";
//-------------------------database_SQL_END-------------------------------------
//-----------------------database_declaration-----------------------------------
int filedb_create(void);
int record_insert(int id, int addr, int device_handle, long int timestamp,int period,int value,int value1,int state);
int record_update(int addr,int period,int value);
int record_update_state(int addr,int state);
int record_delete(int addr);
int record_query(long int current_time , pfun pf);
void* status_minitor(void* data);
void minitor_sig_hander(int signo);
int time_field_update(int addr, long int timestamp);
//----------------------database_function_define--------------------------------

int filedb_create(void)
{
	char* errMsg= NULL;
	int rc = 0;
	char sqlcmd[512] = {0};

	rc = sqlite3_open(file_database_path, &db);
	if(rc != SQLITE_OK)
	{
		fprintf(stderr, "can't open database:%s\n", sqlite3_errmsg(db));
		SNAI_DEBUG_INFO("can't open database:%s", sqlite3_errmsg(db));
		sqlite3_close(db);
		return -1;
	}
	SNAI_DEBUG_INFO("Open database【%s】Successful！",file_database_path);//调试用
	snprintf(sqlcmd, sizeof(sqlcmd), sql_create_data,"addr");
	rc = sqlite3_exec(db, sqlcmd, NULL, NULL, &errMsg);
	if(SQLITE_OK != rc)
	{
		fprintf(stderr, "can't create file database :%s\n", errMsg);
		sqlite3_free(errMsg);
		sqlite3_close(db);
		return -1;
	}
	SNAI_DEBUG_INFO("Create table【%s】Successful！",file_database_path);//调试用
	snprintf(sqlcmd, sizeof(sqlcmd), sql_empty_data,"device_info");
	rc = sqlite3_exec(db, sqlcmd, NULL, NULL, &errMsg);
	if(SQLITE_OK != rc)
	{
		fprintf(stderr, "can't empty file database :%s\n", errMsg);
		sqlite3_free(errMsg);
		sqlite3_close(db);
		return -1;
	}
	return 0;
}
int record_insert(int id, int addr, int device_handle, long int timestamp,int period,int value,int value1,int state)
{
	int      rc              =  0;
	char*    errMsg          =  NULL;
	char     sqlcmd[512]     =  {0};

	snprintf(sqlcmd, sizeof(sqlcmd), sql_insert_data, id, addr, device_handle, timestamp, period,value,value1,state);//打印错误信息
	rc = sqlite3_exec(db, sqlcmd, NULL, NULL, &errMsg);
	if (SQLITE_OK != rc)
	{
		fprintf(stderr, "can't add record to memory database, sqlcmd=%s, err:%s\n", sqlcmd, errMsg);
		sqlite3_free(errMsg);
		return -1;
	}

	return 0;
}
int record_update(int addr,int period,int value)
{
	int      rc              =  0;
	char*    errMsg          =  NULL;
	char     sqlcmd[512]     =  {0};

	snprintf(sqlcmd, sizeof(sqlcmd), sql_update_data,period,value,addr);
	rc = sqlite3_exec(db, sqlcmd, NULL, NULL, &errMsg);
	if (SQLITE_OK != rc)
	{
		fprintf(stderr, "can't update record to memory database, sqlcmd=%s, err:%s\n", sqlcmd, errMsg);
		sqlite3_free(errMsg);
		return -1;
	}

	return 0;
}
int record_update_state(int addr,int state)
{
	int      rc              =  0;
	char*    errMsg          =  NULL;
	char     sqlcmd[512]     =  {0};

	snprintf(sqlcmd, sizeof(sqlcmd), sql_update_state,state,addr);
	rc = sqlite3_exec(db, sqlcmd, NULL, NULL, &errMsg);
	if (SQLITE_OK != rc)
	{
		fprintf(stderr, "can't update record to memory database, sqlcmd=%s, err:%s\n", sqlcmd, errMsg);
		sqlite3_free(errMsg);
		return -1;
	}

	return 0;
}
int record_delete(int addr)
{
	int      rc              =  0;
	char*    errMsg          =  NULL;
	char     sqlcmd[512]     =  {0};

	snprintf(sqlcmd, sizeof(sqlcmd), sql_delete_data, addr);
	rc = sqlite3_exec(db, sqlcmd, NULL, NULL, &errMsg);
	if (SQLITE_OK != rc)
	{
		fprintf(stderr, "can't delete record %s\n", errMsg);
		sqlite3_free(errMsg);
		return -1;
	}

	return 0;
}
int  record_query(long int current_time , pfun pf)//记录查询
{
	int      rc              = 0;
    char     *errMsg         = NULL;
    char**   pRecord         = NULL;
    int      row             = 0;
    int      column          = 0;
    char     sqlcmd[512]     = {0};

    if(db != NULL)
    {
		snprintf(sqlcmd, sizeof(sqlcmd), sql_search_data, current_time);//update_time <= %d-(period*3)  这里%d为当前时间current_time，两个合在一起
		rc = sqlite3_get_table(db, sqlcmd, &pRecord, &row, &column, &errMsg);//得到一維數組pRecord，此数据符合：记录的时间《=最迟记录时间 也就是迟于允许时间
		if (SQLITE_OK != rc) {
			fprintf(stderr, "can't get table %s\n",  errMsg);
			sqlite3_free(errMsg);
			return -1;
		}
		pf(pRecord,row,column);//调用alarm process_query_result，带入一维数组数据
		sqlite3_free_table(pRecord);
    }

	return 0 ;

}

/*  时间戳更新 */

int time_field_update(int addr, long int timestamp)
{
	int      rc              = 0;
	char*    errMsg          = NULL;
	char     sqlCmd[512]  = {0};

	snprintf(sqlCmd, sizeof(sqlCmd), sql_update_data_time_field, timestamp, addr);
	rc = sqlite3_exec(db, sqlCmd, NULL, NULL, &errMsg);
	if (SQLITE_OK != rc)
	{
		fprintf(stderr, "cat't update record %s\n", errMsg);
		sqlite3_free(errMsg);
		return -1;
	}

	return 0;
}

void filedb_close(void)
{
	sqlite3_close(db);
}
//---------------------database_function_define_end-----------------------------
//-------------------------------Alarm------------------------------------------
void process_query_result(char** pr, int row, int column)
{
	int i, j;
	int addr = 0;
	int state = 0;
	unsigned char datatype;
	char datavalue[30];	
	for (i = 1; i <= row; i++) //每一行查找
	{
		j = i * column; //得到每一行的首地址
		state = atoi(pr[j + 7]); //获取当前行的设备状态值(首地址+7列=第二行首地址)
		datatype = atoi(pr[j]); //设备类型
		addr = atoi(pr[j + 1]); //设备地址
		if(state == 1)
		{	
			switch(datatype)
			{
				case SNAI_TYPE_Boiler:
	            case SNAI_TYPE_tmpt:
	            case SNAI_TYPE_out_tmpt:
					if(addr == 2 || addr == 3 || addr == 4 || addr == 5)//其他功能
					{			
						strcpy(SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_NAME[SNAI_DEVICE_EVN_REPOR.Event_Name_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[addr]]],"Faultreport");
						dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[addr]]].type = LEDA_TYPE_ENUM;
						strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[addr]]].key ,"error");
						strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[addr]]].value ,"0");//0离线1故障
						SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_TRIGGER[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[addr]] |= 1;
					}
					break;
				case SNAI_TYPE_NH3:
					break;
				case SNAI_TYPE_Co2:
					break;
				case SNAI_TYPE_Co:
					break;
				case SNAI_TYPE_Illumination:
					break;
				case SNAI_TYPE_Water_Meter:
					break;
				case SNAI_TYPE_Negative_Pressure:
					break;
				case SNAI_TYPE_Position:
					break;
				case SNAI_TYPE_Other:
					break;
				case SNAI_TYPE_Wind_Direction:
					break;
				case SNAI_TYPE_Wind_Speed:
					break;
				default:
					break;

			}
			strcpy(SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_NAME[SNAI_DEVICE_EVN_REPOR.Event_Name_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[addr]]],"Faultreport");
			dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[addr]]].type = LEDA_TYPE_ENUM;
			strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[addr]]].key ,"error");
			strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[addr]]].value ,"0");//0离线1故障
			SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_TRIGGER[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[addr]] |= 1;
			SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[addr]] = 1;//强制启动上报故障
			struct tm *p_tm;
			time_t timestamp;
			timestamp = time((time_t *) NULL);
			p_tm = localtime(&timestamp);
			sprintf(datavalue, "%02d%02d%02d", p_tm->tm_year + 1900,
					p_tm->tm_mon + 1, p_tm->tm_mday);
			SNAI_DEBUG_INFO("Device【%d】The offline time:%02d:%02d:%02d\n\n",addr, p_tm->tm_hour,
					p_tm->tm_min, p_tm->tm_sec);
			record_update_state(addr, 0);
		}
	}
}

/* SIG_KILL 处理函数 */

void minitor_sig_hander(int signo)
{
	SNAI_DEBUG_INFO("Status Monitor thread was killed");
	pthread_exit(0);
}

/* 每隔10s，轮询一下数据库，判断是否有离线设备 */

void* status_minitor(void* data)
{
	signal(SIGKILL,minitor_sig_hander);
	while(1)
	{
		if(SNAI_Gateway_Power_Switch_Flag != 0)
		{
			record_query(time((time_t *)NULL ), process_query_result);//記錄查詢
		}
		sleep(10);
	}
	return NULL;
}

//-----------------------------Alarm——end---------------------------------------
/**
* 判断字符串是否为空
* 为空返回true,不为空返回false
**/

bool SNAI_str_empty(const char *string)
{
  return NULL == string || 0 == strlen(string);
}
 
/**
* 向链表添加section,key,value
* 如果添加时不存在section则新增一个
* 如果对应section的key不存在则新增一个
* 如果section已存在则不会重复创建
* 如果对应section的key已存在则只会覆盖key的值
**/
bool SNAI_cnf_add_option(SNAI_Config *cnf, const char *section, const char *key, const char *value)
{
  if (NULL == cnf || SNAI_str_empty(section) || SNAI_str_empty(key) || SNAI_str_empty(value)) {
    return false; /* 参数不正确,返回false */
  }
  
  SNAI_Data *p = cnf->data; /* 让变量p循环遍历data,找到对应section */
  while (NULL != p && 0 != strcmp(p->section, section)) {
    p = p->next;
  }
  
  if (NULL == p) { /* 说明没有找到section,需要加一个 */
    SNAI_Data *ps = (SNAI_Data*)malloc(sizeof(SNAI_Data));
    if (NULL == ps) {
      exit(-1); /* 申请内存错误 */
    }
    strcpy(ps->section, section);
    ps->option = NULL;    /* 初始的option要为空 */
    ps->next = cnf->data; /* cnf->data可能为NULL */
    cnf->data = p = ps;   /* 头插法插入链表 */
  }
  
  SNAI_Option *q = p->option;
  while (NULL != q && 0 != strcmp(q->key, key)) {
    q = q->next; /* 遍历option,检查key是否已经存在 */
  }
  
  if (NULL == q) { /* 不存在option,则新建一个 */
    q = (SNAI_Option*)malloc(sizeof(SNAI_Option));
    if (NULL == q) {
      exit(-1); /* 申请内存错误 */
    }
    strcpy(q->key, key);
    q->next = p->option; /*这里p->option可能为NULL,不过也没关系 */
    p->option = q; /* 头插法插入链表 */
  }
  strcpy(q->value, value); /* 无论如何要把值改了 */
  
  return true;
}
 
/**
* 去掉字符串内所有空白
* 且忽略注释部分
* 最终得到没有空白的字符串
**/
bool SNAI_strip_comments(char *string, char comment)
{
  if (NULL == string || '\n' == *string || '\r' == *string) {
    return false; /* 第一个字符为回车或换行,表示空行 */
  }
  
  char *p, *q; /* 下面去掉字符串中所有空白字符 */
  for (p = q = string; *p != '\0' && *p != comment; p++) {
    if (0 == isspace(*p)) {
      *q++ = *p; /* 不是空白字符则重写字符串 */
    }
  }
  *q = '\0';
  
  return 0 != strlen(string); /* 字符串长度不为0,表示数据可用 */
}
 
/**
* 传递配置文件路径
* 参数有文件路径,注释字符,分隔符
* 返回Config结构体
**/
SNAI_Config *SNAI_cnf_read_config(const char *filename, char comment, char separator)
{
  SNAI_Config *cnf     = (SNAI_Config*)malloc(sizeof(SNAI_Config));
  cnf->comment    = comment; /* 每一行该字符及以后的字符将丢弃 */
  cnf->separator  = separator; /* 用来分隔Section 和 数据 */
  cnf->data       = NULL; /* 初始数据为空 */
  
  if (SNAI_str_empty(filename)) {
    return cnf; /* 空字符串则直接返回对象 */
  }
  FILE *fp = NULL;
  char *p, sLine[MAX_VALUE];    /* 保存一行数据到字符串 */
  char section[MAX_VALUE], key[MAX_VALUE], value[MAX_VALUE]; /* 缓存section,key,value */
	if(access("/linkedge/run/SNAI_cnf.ini",F_OK | W_OK | R_OK) != -1)//如果配置文件存在，则读取文件配置项
	{
			fp = fopen(filename, "r");
			if(NULL == fp) {
				PRINT_ERRMSG("fopen");
				exit(errno); /* 读文件错误直接按照错误码退出 */
			}
			while (NULL != fgets(sLine, MAX_VALUE, fp)) {
				if (SNAI_strip_comments(sLine, cnf->comment)) { /* 去掉字符串所有空白,注释也忽略 */
				  if ('[' == sLine[0] && ']' == sLine[strlen(sLine)-1]) {
				    memset(section, '\0', MAX_VALUE); /* 清空section,因为strncpy不追加'\0' */
				    strncpy(section, sLine+1, strlen(sLine)-2);
				  } else if (NULL != (p = strchr(sLine, cnf->separator))) {  /* 存在分隔符 */
				    memset(key,   '\0', MAX_VALUE); /* 清空key,因为strncpy不追加'\0' */
				    strncpy(key,  sLine, p - sLine);
				    strcpy(value, p + 1); /* strcpy会追加'\0',所以妥妥哒 */
				    SNAI_cnf_add_option(cnf, section, key, value); /* 添加section,key,value */
				  } /* 如果该行不存在分隔符则忽略这一行 */
				} /* end SNAI_strip_comments */
			} /* end while */
			SNAI_DEBUG_INFO("读取完成！");//调试用
  }
	else
	{
			SNAI_DEBUG_INFO("/linkedge/run/SNAI_cnf.ini 文件不存在,创建中...");//调试用
			fp = fopen("/linkedge/run/SNAI_cnf.ini", "a+");
			if(NULL == fp)
			{
				PRINT_ERRMSG("fopen");
				exit(errno); /* 读文件错误直接按照错误码退出 */
			}
			SNAI_cnf_add_option(cnf, "485_Gateway", "Power_Switch", "1");
			SNAI_cnf_add_option(cnf, "485_Gateway", "Main_Switch_Filtration", "1");
			SNAI_cnf_add_option(cnf, "485_Gateway", "Period_Time", "10");
			SNAI_cnf_add_option(cnf, "485_Gateway", "Over_Timeout", "30");

			SNAI_cnf_add_option(cnf, "485_DEV_0", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_0", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_0", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_0", "Alarm_Temp", "20.0");
			SNAI_cnf_add_option(cnf, "485_DEV_0", "Alarm_Humi", "80.0");

			SNAI_cnf_add_option(cnf, "485_DEV_1", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_1", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_1", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_1", "Alarm_Temp", "30.0");
			SNAI_cnf_add_option(cnf, "485_DEV_1", "Alarm_Humi", "80.0");

			SNAI_cnf_add_option(cnf, "485_DEV_2", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_2", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_2", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_2", "Alarm_Temp", "30.0");
			SNAI_cnf_add_option(cnf, "485_DEV_2", "Alarm_Humi", "80.0");

			SNAI_cnf_add_option(cnf, "485_DEV_3", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_3", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_3", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_3", "Alarm_Temp", "30.0");
			SNAI_cnf_add_option(cnf, "485_DEV_3", "Alarm_Humi", "80.0");

			SNAI_cnf_add_option(cnf, "485_DEV_4", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_4", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_4", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_4", "Alarm_Temp", "40.0");
			SNAI_cnf_add_option(cnf, "485_DEV_4", "Alarm_Humi", "80.0");

			SNAI_cnf_add_option(cnf, "485_DEV_5", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_5", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_5", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_5", "Alarm_Temp", "50.0");
			SNAI_cnf_add_option(cnf, "485_DEV_5", "Alarm_Humi", "80.0");

			SNAI_cnf_add_option(cnf, "485_DEV_6", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_6", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_6", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_6", "Alarm_Boiler", "60.0");

			SNAI_cnf_add_option(cnf, "485_DEV_7", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_7", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_7", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_7", "Alarm_Negative_pressure", "5");

			SNAI_cnf_add_option(cnf, "485_DEV_8", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_8", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_8", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_8", "Alarm_Co2", "800");

			SNAI_cnf_add_option(cnf, "485_DEV_9", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_9", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_9", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_9", "Alarm_NH3", "800");

			SNAI_cnf_add_option(cnf, "485_DEV_10", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_10", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_10", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_10", "Alarm_iLLumination", "200");

			SNAI_cnf_add_option(cnf, "485_DEV_11", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_11", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_11", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_11", "Alarm_Position_Value_L", "600");

			SNAI_cnf_add_option(cnf, "485_DEV_12", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_12", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_12", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_12", "Alarm_Position_Value_R", "600");

			SNAI_cnf_add_option(cnf, "485_DEV_13", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_13", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_13", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_13", "Alarm_Co", "800");

			SNAI_cnf_add_option(cnf, "485_DEV_14", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_14", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_14", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_14", "Alarm_14", "800");

			SNAI_cnf_add_option(cnf, "485_DEV_15", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_15", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_15", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_15", "Alarm_15", "800");

			SNAI_cnf_add_option(cnf, "485_DEV_16", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_16", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_16", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_16", "Alarm_16", "800");
			
			SNAI_cnf_add_option(cnf, "485_DEV_17", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_17", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_17", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_17", "Alarm_Water", "99.9");

			SNAI_cnf_add_option(cnf, "485_DEV_18", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_18", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_18", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_18", "Alarm_18", "800");

			SNAI_cnf_add_option(cnf, "485_DEV_19", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_19", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_19", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_19", "Alarm_19", "800");

			SNAI_cnf_add_option(cnf, "485_DEV_20", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_20", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_20", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_20", "Alarm_Temp", "30.0");
			SNAI_cnf_add_option(cnf, "485_DEV_20", "Alarm_Humi", "80.0");

			SNAI_cnf_add_option(cnf, "485_DEV_21", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_21", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_21", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_21", "Alarm_Temp", "30.0");
			SNAI_cnf_add_option(cnf, "485_DEV_21", "Alarm_Humi", "80.0");

			SNAI_cnf_add_option(cnf, "485_DEV_22", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_22", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_22", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_22", "Alarm_Temp", "30.0");
			SNAI_cnf_add_option(cnf, "485_DEV_22", "Alarm_Humi", "80.0");

			SNAI_cnf_add_option(cnf, "485_DEV_23", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_23", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_23", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_23", "Alarm_Temp", "30.0");
			SNAI_cnf_add_option(cnf, "485_DEV_23", "Alarm_Humi", "80.0");

			SNAI_cnf_add_option(cnf, "485_DEV_24", "Power_Switch", "0");
			SNAI_cnf_add_option(cnf, "485_DEV_24", "Period_Time", "15000");
			SNAI_cnf_add_option(cnf, "485_DEV_24", "Over_Timeout", "30");
			SNAI_cnf_add_option(cnf, "485_DEV_24", "Alarm_Temp", "30.0");
			SNAI_cnf_add_option(cnf, "485_DEV_24", "Alarm_Humi", "80.0");

			SNAI_cnf_add_option(cnf, "CONFIG_FLAG", "CONFIG_ENABLE", "0");
			SNAI_cnf_add_option(cnf, "CONFIG_FLAG", "SNAI_LED_Ver","V1.0XXXX");

			SNAI_cnf_write_file(cnf, "/linkedge/run/SNAI_cnf.ini", "SNAI 485 DEVICE DRIVER CONFIG INFO"); // 将对象写入/linkedge/run/SNAI_cnf.ini文件
			SNAI_DEBUG_INFO("写入完成！");//调试用
			SNAI_print_config(cnf);//调试用
	}
  fclose(fp);
	SNAI_DEBUG_INFO("关闭文件！");//调试用
  return cnf;
}
 
/**
* 获取指定类型的值
* 根据不同类型会赋值给对应值
* 本方法需要注意,int和double的转换,不满足就是0
*     需要自己写代码时判断好
**/
bool SNAI_cnf_get_value(SNAI_Config *cnf, const char *section, const char *key)
{
  SNAI_Data *p = cnf->data; /* 让变量p循环遍历data,找到对应section */
  while (NULL != p && 0 != strcmp(p->section, section)) {
    p = p->next;
  }
  
  if (NULL == p) {
    SNAI_DEBUG_INFO("section not find!");//调试用
    return false;
  }
  
  SNAI_Option *q = p->option;
  while (NULL != q && 0 != strcmp(q->key, key)) {
    q = q->next; /* 遍历option,检查key是否已经存在 */
  }
  
  if (NULL == q) {
    SNAI_DEBUG_INFO("key not find!");//调试用
    return false;
  }
  
  strcpy(cnf->re_string, q->value);       /* 将结果字符串赋值 */
  cnf->re_int    = atoi(cnf->re_string);  /* 转换为整形 */
  cnf->re_bool   = 0 == strcmp ("true", cnf->re_string); /* 转换为bool型 */
  cnf->re_double = atof(cnf->re_string);  /* 转换为double型 */
  
  return true;
}
 
/**
* 判断section是否存在
* 不存在返回空指针
* 存在则返回包含那个section的Data指针
**/
SNAI_Data *SNAI_cnf_has_section(SNAI_Config *cnf, const char *section)
{
  SNAI_Data *p = cnf->data; /* 让变量p循环遍历data,找到对应section */
  while (NULL != p && 0 != strcmp(p->section, section)) {
    p = p->next;
  }
  
  if (NULL == p) { /* 没找到则不存在 */
    return NULL;
  }
  
  return p;
}
 
/**
* 判断指定option是否存在
* 不存在返回空指针
* 存在则返回包含那个section下key的Option指针
**/
SNAI_Option *SNAI_cnf_has_option(SNAI_Config *cnf, const char *section, const char *key)
{
  SNAI_Data *p = SNAI_cnf_has_section(cnf, section);
  if (NULL == p) { /* 没找到则不存在 */
    return NULL;
  }
  
  SNAI_Option *q = p->option;
  while (NULL != q && 0 != strcmp(q->key, key)) {
    q = q->next; /* 遍历option,检查key是否已经存在 */
  }
  if (NULL == q) { /* 没找到则不存在 */
    return NULL;
  }
  
  return q;
}
 
/**
* 将Config对象写入指定文件中
* header表示在文件开头加一句注释
* 写入成功则返回true
**/
bool SNAI_cnf_write_file(SNAI_Config *cnf, const char *filename, const char *header)
{
  FILE *fp = fopen(filename, "w");
  if(NULL == fp) {
    SNAI_DEBUG_INFO("fopen");//调试用
    exit(errno); /* 读文件错误直接按照错误码退出 */
  }
  
  if (0 < strlen(header)) { /* 文件注释不为空,则写注释到文件 */
    fprintf(fp, "%c %s\n\n", cnf->comment, header);
  }
  
  SNAI_Option *q;
  SNAI_Data   *p = cnf->data;
  while (NULL != p) {
    fprintf(fp, "[%s]\n", p->section);
    q = p->option;
    while (NULL != q) {
      fprintf(fp, "%s %c %s\n", q->key, cnf->separator, q->value);
      q = q->next;
    }
    p = p->next;
  }
  
  fclose(fp);
  return true;
}
 
/**
* 删除option
**/
bool SNAI_cnf_remove_option(SNAI_Config *cnf, const char *section, const char *key)
{
  SNAI_Data *ps = SNAI_cnf_has_section(cnf, section);
  if (NULL == ps) { /* 没找到则不存在 */
    return NULL;
  }
  
  SNAI_Option *p, *q;
  q = p = ps->option;
  while (NULL != p && 0 != strcmp(p->key, key)) {
    if (p != q) { q = q->next; } /* 始终让q处于p的上一个节点 */
    p = p->next;
  }
  
  if (NULL == p) { /* 没找到则不存在 */
    return NULL;
  }
  
  if (p == q) { /* 第一个option就匹配了 */
    ps->option = p->next;
  } else {
    q->next = p->next;
  }
  
  free(p);
  q = p = NULL; // 避免野指针
  
  return true;
}
 
/**
* 删除section
**/
bool SNAI_cnf_remove_section(SNAI_Config *cnf, const char *section)
{
  if (SNAI_str_empty(section)) {
    return false;
  }
  
  SNAI_Data *p, *q;
  q = p = cnf->data; /* 让变量p循环遍历data,找到对应section */
  while (NULL != p && 0 != strcmp(p->section, section)) {
    if (p != q) { q = q->next; } /* 始终让q处于p的上一个节点 */
    p = p->next;
  }
  
  if (NULL == p) { /* 没有找到section */
    return false;
  }
  
  if (p == q) { /* 这里表示第一个section,因此链表头位置改变 */
    cnf->data = p->next;
  } else { /* 此时是中间或尾部节点 */
    q->next = p->next;
  }
  
  SNAI_Option *o = p->option;
  while (NULL != o) {
    free(o); /* 循环释放所有option */
    o = o->next;
  }
  p->option = NULL; // 避免野指针
  free(p); /* 释放删除的section */
  q = p = NULL;  // 避免野指针
  
  return true;
}
 
/**
* 打印当前Config对象
**/
void SNAI_print_config(SNAI_Config *cnf)
{
  SNAI_Data *p = cnf->data; // 循环打印结果
  while (NULL != p) {
    SNAI_DEBUG_INFO("[%s]",p->section);
    
    SNAI_Option *q = p->option;
    while (NULL != q) {
      SNAI_DEBUG_INFO("  %s %c %s", q->key, cnf->separator, q->value);
      q = q->next;
    }
    p = p->next;
  }
}

/*
清空配置内存
*/
void SNAI_delete_config(SNAI_Config *cnf)
{
	SNAI_Data *p,*q;//p = next
	SNAI_Option *op,*oq;//op = next
  p = q = cnf->data; /* 让变量p循环遍历data*/
  while (NULL != q) {
    p = q->next;/*下个要删除的赋值*/
		op = oq = q->option;
 	  while (NULL != oq) {
			op = oq->next;
		  free(oq);   /* 循环释放所有option 头 */
		  oq = op;    /*下一个头*/
  	}
		free(q);    /*删除本次节点*/
		q = p;      /*下一个送到断头台*/
  }
	cnf->data = NULL;
  q = p = NULL; // 避免野指针
	op = oq = NULL; // 避免野指针
}

/*多线程结束操作*/
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
    if(SNAI_Origin_Day != tm_time.tm_mday && tm_time.tm_hour == 0)
		{		
			int status = unlink(SNAI_LOG_File_TIME);//清除前一天日志文件
			if (0 != status)
			{
				printf("unlink failed \n");
				//abort();
			}
			SNAI_log_init();//new day
		}
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
	pthread_mutex_destroy(&SNAI_Decode_mutex_lock);
	pthread_cond_destroy(&SNAI_cond);
	pthread_opt_seq_kill(opt_seq_ptr);
}
void SNAI_driver_exit(int xx)
{
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

	status = ftruncate(fd, cb->count);//指定的文件大小改为参数length 指定的大小
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
/*同步一次*/
/*void cb_read_offset_sync(SNAI_circular_buffer *cb)
{
    cb->read_offset = cb->write_offset;
}*/
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
	struct tm  *tp;  
	time_t t = time(NULL); 
	tp = localtime(&t);
		SNAI_Origin_Day = tp->tm_mday;//首次 
    sprintf(SNAI_LOG_File_TIME,"/dev/shm/SNAI_LOG-%d-%u-%d-%d-%d.txt",tp->tm_year+1900,tp->tm_mon+1,tp->tm_mday 
   			,tp->tm_hour,tp->tm_min); 
		if((SNAI_log_id = open(SNAI_LOG_File_TIME,O_CREAT | O_RDWR | O_APPEND | O_NONBLOCK,S_IWGRP | S_IROTH ))<0)//创建日志文件
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
	unsigned char SNAI_CRC_value_L = 0;
	unsigned char SNAI_CRC_value_H = 0;
    while (1)
    {
		//SNAI_DEBUG_INFO("上锁！");//调试用
		pthread_mutex_lock(&SNAI_Decode_mutex_lock);
		if(SNAI_Gateway_Power_Switch_Flag != 0)
		{
	        //printf("pthread %d is aliver",1);//调试用
	        //SNAI_DEBUG_INFO("进入解析！");//调试用
	        if (cb_bytes_can_read(cb) >= SNAI_MSG_MIN_LEN) //比较当前可读数据长度=当前写入长度-已读长度）大于8即可进入
	        {
	            msg_type = *((unsigned char*) (cb->ptr + cb->read_offset)); //读取数据头，含义为消息类型
	            msg_type2 = *((unsigned char*) (cb->ptr + cb->read_offset+1)); //读取数据头，含义为消息类型
	            msg_len = *((unsigned char*) (cb->ptr + cb->read_offset + 2)); //第二个字节为数据长度
	            msg_len_water_meter = (*((unsigned char*) (cb->ptr + cb->read_offset + 2)))+5; //水表情况下，数据长度

	//假设读取0-5数据地址=则长度为6个字节 6作为下一次读取地址起始位OK
	            if ( ((msg_type != SNAI_MSG_TYPE_DATA) || (msg_type2 != SNAI_MSG_TYPE_DATA2)) 
										&& ((msg_type != 0x11) || (msg_len_water_meter != 13)) )
	                             //头不是0xFF and not is 0x11，或者长度小于8。皆属于解码错误
	            {
									//SNAI_DEBUG_INFO("解码错误！");//调试用
	                //printf("pre decode err !\r\n");//调试用
	                //printf("msg_type:%02X msg_len:%u\r\n", msg_type, msg_len);//调试用
	                cb_read_offset_inc(cb, 1);
	            } else { //类型正确的情况下，或者长度正确的情况下
	                while (cb_bytes_can_read(cb) < msg_len)
	                {
											//SNAI_DEBUG_INFO("小于可读等待中......");//调试用
	 										sleep(1);//秒级挂起线程
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
									if(msg_type != 0x11 && msg_len_water_meter != 13)
									{
					            if((SNAI_CRC_value_L == *((unsigned char*) (cb->ptr + cb->read_offset + msg_len-2))) && (SNAI_CRC_value_H == *((unsigned char*) (cb->ptr + cb->read_offset + msg_len-1))))//校验CRC
					            {
					                for (i = 0; i < msg_len; i++)
					                {
					                        //printf("%02X ",
					                                        //*((unsigned char*) (cb->ptr + cb->read_offset + i))); //打印2位的16进制数，不足位0补齐。//调试用
					                        SNAI_all_device_value[i] = *((unsigned char*) (cb->ptr + cb->read_offset + i));//当前数据给数组。
					                        //SNAI_DEBUG_INFO("接收数据：%02X",SNAI_all_device_value[i]);//调试用
					                }
					                //printf("\n");//调试用
									        if ((SNAI_READ_DATA == SNAI_all_device_value[4]) && (msg_len > SNAI_MSG_MIN_LEN))
									        {
									            //SNAI_DEBUG_INFO("解析传感数据值-->");//调试用
									            tlv_decode(cb);
									            cb_read_offset_inc(cb, msg_len);
															
									        }
					            }
											else
											{
													//SNAI_DEBUG_INFO("传感器校验CRC失败！");//调试用
													cb_read_offset_inc(cb, 1);
											}
									}
									else
									{
					            if((SNAI_CRC_value_L == *((unsigned char*) (cb->ptr + cb->read_offset + msg_len_water_meter-2))) && (SNAI_CRC_value_H == *((unsigned char*) (cb->ptr + cb->read_offset + msg_len_water_meter-1))))//校验CRC
					            {
										
					                for (i = 0; i < msg_len_water_meter; i++)
					                {
					                        printf("%02X ",
					                                        *((unsigned char*) (cb->ptr + cb->read_offset + i))); //打印2位的16进制数，不足位0补齐。
					                        SNAI_all_device_value[i] = *((unsigned char*) (cb->ptr + cb->read_offset + i));//当前数据给数组。
					                        //SNAI_DEBUG_INFO("接收水表数据：%02X",SNAI_all_device_value[i]);//调试用
					                }
					                printf("\n");
									        if ((0x11 == SNAI_all_device_value[0]) && (msg_len_water_meter > SNAI_MSG_MIN_LEN))
									        {
									            //SNAI_DEBUG_INFO("解析水表传感数据值-->");//调试用
									            tlv_decode(cb);
									            cb_read_offset_inc(cb, msg_len_water_meter);
									        }
					            }
											else
											{
													//SNAI_DEBUG_INFO("水表校验CRC失败！");//调试用
													cb_read_offset_inc(cb, 1);
											}
	                }
	                //SNAI_DEBUG_INFO("读取地址-->"%lu""%lu"<--写入地址",cb->read_offset,cb->write_offset);//调试用									
	            }
	        }
        }
				//SNAI_DEBUG_INFO("激活条件变量，调用上报数据线程！");//调试用
				pthread_cond_signal(&SNAI_cond);//先解析完，再激活上传线程
				//SNAI_DEBUG_INFO("解锁！");//调试用
				pthread_mutex_unlock(&SNAI_Decode_mutex_lock);
				//SNAI_DEBUG_INFO("解锁完毕！挂起本线程500ms");//调试用
				usleep(100000);//us级挂起线程
    }
    return NULL;
}

/*0XFF 0XEE LEN  485_ADD 0X52/0X51 TYPE1 DATA_LEN H L TYPE2 DATA_LEN2 H L CRC_L CRC_H*/
/* 1     1   1      1        1       1       1    1 1   ?       ?     ? ?   1     1  */
/*Water_Meter*/
/*485ADDR FUNC LEN DATA0 ---DATA7 CRC_L CRC_H*/
/*设备解析、数据上报
1、根据485地址找到自身句柄号
2、依据句柄号，找到自身数据存放起始地址【上报使用句柄】
3、从起始地址开始、顺序存放数据
4、事件最多三个 （2.3.4.5设备用）
5、每个事件最多2个参数
6、每个事件及参数顺序如下：故障及参数/报警1及参数1&2/报警2及参数1&2
7、每个设备具有独立8位事件位，地位开始LSB位故障事件位/报警1事件/报警2事件...
*/
void tlv_decode(SNAI_circular_buffer *cb)
{
    float float_temp = 0,sym_bit = 0,float_Humi_temp = 0;
	double flow_rate_value = 0,Water_Yield = 0;
    unsigned short Current_Co2_value = 0,Current_NH3_value = 0,Current_light_intensity = 0,Current_Position_value = 0,
            Current_Co_value = 0,Current_Negative_Pressure_value = 0;
	time_t timestamp;
	timestamp = time((time_t *) NULL);
    if(SNAI_all_device_value[0] == 0x11 && SNAI_all_device_value[1] == 0x03 && SNAI_all_device_value[2] == 0x08)
    {
		//SNAI_DEBUG_INFO("解析【%u】中......",SNAI_all_device_value[0]);//调试用
        dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[0]]]].type = LEDA_TYPE_DOUBLE;
        strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[0]]]].key ,"CurrentWater_Yield");
        sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[0]]]].value,"%02X%02X.%02X%02X",SNAI_all_device_value[3],SNAI_all_device_value[4],SNAI_all_device_value[5],SNAI_all_device_value[6]);
		/*SNAI_DEBUG_INFO("获取17#水量值:【%02X%02X.%02X%02Xm³】瞬时流量:【%02X%02X%02X.%02XL/H】",
			SNAI_all_device_value[3],SNAI_all_device_value[4],SNAI_all_device_value[5],SNAI_all_device_value[6]
                ,SNAI_all_device_value[7],SNAI_all_device_value[8],SNAI_all_device_value[9],SNAI_all_device_value[10]);*/
        Water_Yield =(double)((SNAI_all_device_value[3]>>4)*1000+(SNAI_all_device_value[3]&0x0F)*100+
									(SNAI_all_device_value[4]>>4)*10+(SNAI_all_device_value[4]&0x0F)+
									(SNAI_all_device_value[5]>>4)*0.1+(SNAI_all_device_value[5]&0x0F)*0.01+
									(SNAI_all_device_value[6]>>4)*0.001+(SNAI_all_device_value[6]&0x0F)*0.0001);
        flow_rate_value = (double)((SNAI_all_device_value[7]>>4)*100000+(SNAI_all_device_value[7]&0x0F)*10000+
									(SNAI_all_device_value[8]>>4)*1000+(SNAI_all_device_value[8]&0x0F)*100+
									(SNAI_all_device_value[9]>>4)*10+(SNAI_all_device_value[9]&0x0F)+
									(SNAI_all_device_value[10]>>4)*0.1+(SNAI_all_device_value[10]&0x0F)*0.01);
        flow_rate_value = flow_rate_value*3600.00;//L/H transform L/s
        dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[0]]]+1].type = LEDA_TYPE_DOUBLE;
        strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[0]]]+1].key ,"CurrentFlow_Rate");
        sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[0]]]+1].value,"%.2f",flow_rate_value);
		if(SNAI_Period_Set.SNAI_Main_Filtration_Flag == 0)
		{
			SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[0]]] = 1;
		}
		else//启用过滤
		{
			SNAI_RS485_DATA_Filtration(SNAI_all_device_value[0],&Water_Yield,0,SNAI_TYPE_Water_Meter);
		}
		if(Water_Yield >= SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_Water)//水量报警判断
		{
			//SNAI_DEBUG_INFO("设备:【%u】当前值【%.4fm³】临界值【%.1fm³】开启高水量报警！",SNAI_all_device_value[0],Water_Yield,SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_Water);//调试用
			strcpy(SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_NAME[SNAI_DEVICE_EVN_REPOR.Event_Name_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[0]]]+1],"Water_Yiled_Alarm");
			dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[0]]]+1].type = LEDA_TYPE_ENUM;
			dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[0]]]+2].type = LEDA_TYPE_DOUBLE;
			strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[0]]]+2].key ,"CurrentWater_Yield");
			sprintf(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[0]]]+2].value ,"%s",dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[0]]]].value);
			strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[0]]]+1].key ,"AlarmType");
        	strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[0]]]+1].value ,"1");//0正常/1高量/2低量
        	SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_TRIGGER[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[0]]] |= 2;//允许上报事件
		}
		/*else 
		{
			strcpy(SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_NAME[SNAI_DEVICE_EVN_REPOR.Event_Name_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[0]]]],"Faultreport");
			dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[0]]]].type = LEDA_TYPE_ENUM;
			strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[0]]]].key ,"error");
			strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[0]]]].value ,"0");//0正常1故障
			SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_TRIGGER[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[0]]] |= 1;
		}*/
		record_insert(SNAI_TYPE_Water_Meter, SNAI_all_device_value[0], SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[0]], timestamp,SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_all_device_value[0]],(int)Water_Yield,(int)flow_rate_value,1);
		record_update(SNAI_all_device_value[0],SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_all_device_value[0]],(int)Water_Yield);
		time_field_update(SNAI_all_device_value[0], timestamp);
		record_update_state(SNAI_all_device_value[0], 1);
    }
    else//
    {
		//SNAI_DEBUG_INFO("解析【%u】中......",SNAI_all_device_value[3]);//调试用
        switch (SNAI_all_device_value[5])//先匹配数据类型 再匹配 地址-->方便后期增设同类型设备
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
                    	if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[SNAI_all_device_value[3]] == 1)
                    	{
	                        dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].type = LEDA_TYPE_DOUBLE;//LEDA_TYPE_TEXT;//LEDA_TYPE_FLOAT;//浮点
	                        strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].key ,"CurrentTemperature");
	                        sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].value,"%.1f",float_temp);
	                        //gcvt(float_temp, 3, dev_proper_data[0].value);
	                        //SNAI_DEBUG_INFO("获取%u#水温数负值【%.1f℃】",SNAI_all_device_value[3],float_temp);//调试用
	                        
                    	}
                    	break;
                    case SNAI_TYPE_tmpt:
						if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[SNAI_all_device_value[3]] == 1)
						{
							dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].type = LEDA_TYPE_DOUBLE;//LEDA_TYPE_TEXT;//LEDA_TYPE_FLOAT;//浮点
							strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].key ,"CurrentTemperature");
							sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].value,"%.1f",float_temp);
							//SNAI_DEBUG_INFO("获取%u#温度负值【%.1f℃】",SNAI_all_device_value[3],float_temp);//调试用
							
						}
						break;
                    case SNAI_TYPE_out_tmpt:
                        if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[SNAI_all_device_value[3]] == 1)
                        {
                            dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].type = LEDA_TYPE_DOUBLE;
                            strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].key ,"CurrentTemperature");
                            sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].value,"%.1f",float_temp);
                            //SNAI_DEBUG_INFO("获取室外%u#温度负值【%.1f℃】",SNAI_all_device_value[3],float_temp);//调试用
                            
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
						if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[SNAI_all_device_value[3]] == 1)
                    	{
	                        dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].type = LEDA_TYPE_DOUBLE;
	                        strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].key ,"CurrentTemperature");
	                        sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].value,"%.1f",float_temp);
	                        //gcvt(float_temp, 3, dev_proper_data[0].value);
	                        //SNAI_DEBUG_INFO("获取%u#水温数正值【%.1f℃】",SNAI_all_device_value[3],float_temp);//调试用
	                        
                    	}
						break;
                    case SNAI_TYPE_tmpt:
						if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[SNAI_all_device_value[3]] == 1)
						{
							dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].type = LEDA_TYPE_DOUBLE;//LEDA_TYPE_TEXT;//LEDA_TYPE_FLOAT;//浮点
							strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].key ,"CurrentTemperature");
							sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].value,"%.1f",float_temp);
							//SNAI_DEBUG_INFO("获取%u#温度正值【%.1f℃】",SNAI_all_device_value[3],float_temp);//调试用							
							
						}
						break;
                    case SNAI_TYPE_out_tmpt:
                        if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[SNAI_all_device_value[3]] == 1)
                        {
                            dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].type = LEDA_TYPE_DOUBLE;
                            strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].key ,"CurrentTemperature");
                            sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].value,"%.1f",float_temp);
                            //SNAI_DEBUG_INFO("获取室外%u#温度正值【%.1f℃】",SNAI_all_device_value[3],float_temp);//调试用
                            
                        }
                        break;
                    default:
                        break;
                    }
                }
				if(SNAI_Period_Set.SNAI_Main_Filtration_Flag == 0)
				{
					SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]] = 1;
				}
				else
				{
					SNAI_RS485_DATA_Filtration(SNAI_all_device_value[3],&float_temp,0,SNAI_TYPE_out_tmpt);//温度检测
				}
				if(float_temp >= SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_TMP[SNAI_all_device_value[3]])//温度报警判断
				{
					//SNAI_DEBUG_INFO("设备:【%u】当前值【%.1f℃】临界值【%.1f℃】开启高温报警！",SNAI_all_device_value[3],float_temp,SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_TMP[SNAI_all_device_value[3]]);//调试用
					strcpy(SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_NAME[SNAI_DEVICE_EVN_REPOR.Event_Name_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1],"Temperature_Alarm");
					dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].type = LEDA_TYPE_ENUM;
					dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].type = LEDA_TYPE_DOUBLE;
					strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].key ,"CurrentTemperature");
					sprintf(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].value ,"%s",dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].value);
					strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].key ,"AlarmType");
		        	strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].value ,"1");//0正常/1高温/2低温
		        	SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_TRIGGER[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]] |= 2;//允许上报事件
				}
				if(SNAI_all_device_value[5] != SNAI_TYPE_Boiler)//防水温进入
				{
	                switch(SNAI_all_device_value[9])//湿度解析
	                {
	                    case SNAI_TYPE_humi:
	                    if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[SNAI_all_device_value[3]] == 1)
	                    {
	                        dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].type = LEDA_TYPE_DOUBLE;//浮点
	                        strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].key ,"CurrentHumidity");
	                        sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].value,"%.1f",float_Humi_temp);
	                        //SNAI_DEBUG_INFO("获取%u#湿度值【%.1f%%】",SNAI_all_device_value[3],float_Humi_temp);//调试用
	                        
	                    }
	                        break;
	                    case SNAI_TYPE_out_humi:
	                    if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[SNAI_all_device_value[3]] == 1)
	                    {
	                        dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].type = LEDA_TYPE_DOUBLE;//浮点
	                        strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].key ,"CurrentHumidity");
	                        sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].value,"%.1f",float_Humi_temp);
	                        //SNAI_DEBUG_INFO("获取%u#室外湿度值【%.1f%%】",SNAI_all_device_value[3],float_Humi_temp);//调试用
	                        
	                    }
		                   break;
		                default:
		                   break;
	                }
					if(SNAI_Period_Set.SNAI_Main_Filtration_Flag == 0)
					{
						SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]] = 1;
					}
					else
					{
						SNAI_RS485_DATA_Filtration(SNAI_all_device_value[3],&float_Humi_temp,1,SNAI_TYPE_out_tmpt);//湿度检测
					
					}
					if(float_Humi_temp >= SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_Humi[SNAI_all_device_value[3]])//湿度报警判断
					{
						//SNAI_DEBUG_INFO("设备:【%u】当前值【%.1f℃】临界值【%.1f℃】开启高湿报警！",SNAI_all_device_value[3],float_Humi_temp,SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_Humi[SNAI_all_device_value[3]]);//调试用
						strcpy(SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_NAME[SNAI_DEVICE_EVN_REPOR.Event_Name_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2],"Humidity_Alarm");
						dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+3].type = LEDA_TYPE_ENUM;
						dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+4].type = LEDA_TYPE_DOUBLE;
						strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+4].key ,"CurrentHumidity");
						sprintf(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+4].value ,"%s",dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].value);
						strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+3].key ,"AlarmType");
			        	strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+3].value ,"1");//0正常/1高量/2低量
			        	SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_TRIGGER[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]] |= 4;//允许上报事件
					}
        		}
				record_insert(SNAI_all_device_value[5], SNAI_all_device_value[3], SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]], timestamp,SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_all_device_value[3]],(int)float_temp,(int)float_Humi_temp,1);
				//SNAI_DEBUG_INFO("插入表设备类型:【%u】设备地址【%u】设备线程【%d】当前时间【%ld】设备周期【%d】设备值【%d】设备值【%d】设备状态【1】",SNAI_all_device_value[5], SNAI_all_device_value[3], SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]], timestamp,SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_all_device_value[3]],(int)float_temp,(int)float_Humi_temp);//调试用
				record_update(SNAI_all_device_value[3],SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_all_device_value[3]],(int)float_temp);
            break;
        case SNAI_TYPE_Co2:
            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[SNAI_all_device_value[3]] == 1)
            {
                Current_Co2_value = (SNAI_all_device_value[7] & 0x7F)*256;
                Current_Co2_value += (SNAI_all_device_value[8]>>4)*16+
                                        (SNAI_all_device_value[8]&0x0F);
                dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].type = LEDA_TYPE_INT;
                strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].key ,"GasConcentration");
                sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].value,"%u",Current_Co2_value);
                //SNAI_DEBUG_INFO("获取【%u#】二氧化碳值【%uppm】",SNAI_all_device_value[3],Current_Co2_value);//8#调试用
				if(SNAI_Period_Set.SNAI_Main_Filtration_Flag == 0)
				{
                	SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]] = 1;
				}
				else
				{
                	SNAI_RS485_DATA_Filtration(SNAI_all_device_value[3],&Current_Co2_value,0,SNAI_TYPE_Co2);
				}
				if(Current_Co2_value >= SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[SNAI_all_device_value[3]])//气体浓度报警判断
				{
					//SNAI_DEBUG_INFO("设备:【%u】当前值【%uppm】临界值【%uppm】开启高浓度报警！",SNAI_all_device_value[3],Current_Co2_value,SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[SNAI_all_device_value[3]]);//调试用
					strcpy(SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_NAME[SNAI_DEVICE_EVN_REPOR.Event_Name_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1],"CO2_Alarm");
					dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].type = LEDA_TYPE_ENUM;
					dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].type = LEDA_TYPE_INT;
					strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].key ,"GasConcentration");
					sprintf(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].value ,"%s",dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].value);
					strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].key ,"AlarmType");
		        	strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].value ,"1");//0正常/1高量/2低量
		        	SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_TRIGGER[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]] |= 2;//允许上报事件
				}
				record_insert(SNAI_all_device_value[5], SNAI_all_device_value[3], SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]], timestamp,SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_all_device_value[3]],Current_Co2_value,0,1);
				record_update(SNAI_all_device_value[3],SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_all_device_value[3]],Current_Co2_value);
            }
            break;
        case SNAI_TYPE_NH3:
            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[SNAI_all_device_value[3]] == 1)
            {
                Current_NH3_value = (SNAI_all_device_value[7] & 0x7F)*256;
                Current_NH3_value += (SNAI_all_device_value[8]>>4)*16+
                                        (SNAI_all_device_value[8]&0x0F);
                dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].type = LEDA_TYPE_INT;
                strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].key ,"GasConcentration");
                sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].value,"%u",Current_NH3_value);
                //SNAI_DEBUG_INFO("获取【%u#】氨气值【%uppm】",SNAI_all_device_value[3],Current_NH3_value);//9#调试用
				if(SNAI_Period_Set.SNAI_Main_Filtration_Flag == 0)
				{
                	SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]] = 1;
				}
				else
				{
                	SNAI_RS485_DATA_Filtration(SNAI_all_device_value[3],&Current_NH3_value,0,SNAI_TYPE_NH3);
				}
				if(Current_NH3_value >= SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[SNAI_all_device_value[3]])//气体浓度报警判断
				{
					//SNAI_DEBUG_INFO("设备:【%u】当前值【%uppm】临界值【%uppm】开启高浓度报警！",SNAI_all_device_value[3],Current_NH3_value,SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[SNAI_all_device_value[3]]);//调试用
					strcpy(SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_NAME[SNAI_DEVICE_EVN_REPOR.Event_Name_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1],"NH3_Alarm");
					dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].type = LEDA_TYPE_ENUM;
					dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].type = LEDA_TYPE_INT;
					strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].key ,"GasConcentration");
					sprintf(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].value ,"%s",dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].value);
					strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].key ,"AlarmType");
		        	strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].value ,"1");//0正常/1高量/2低量
		        	SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_TRIGGER[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]] |= 2;//允许上报事件
				}
				record_insert(SNAI_all_device_value[5], SNAI_all_device_value[3], SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]], timestamp,SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_all_device_value[3]],Current_NH3_value,0,1);
				record_update(SNAI_all_device_value[3],SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_all_device_value[3]],Current_NH3_value);
            }
            break;
        case SNAI_TYPE_Illumination:
            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[SNAI_all_device_value[3]] == 1)
            {
                Current_light_intensity = (SNAI_all_device_value[7] & 0x7F)*256;
                Current_light_intensity += (SNAI_all_device_value[8]>>4)*16+
                                        (SNAI_all_device_value[8]&0x0F);
                dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].type = LEDA_TYPE_INT;
                strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].key ,"Light_illumination");
                sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].value,"%u",Current_light_intensity);
                //SNAI_DEBUG_INFO("获取【%u#】光照值【%uLux】",SNAI_all_device_value[3],Current_light_intensity);//10# 调试用
				if(SNAI_Period_Set.SNAI_Main_Filtration_Flag == 0)
				{
                	SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]] = 1;}
				else
				{
                	SNAI_RS485_DATA_Filtration(SNAI_all_device_value[3],&Current_light_intensity,0,SNAI_TYPE_Illumination);
				}
				if(Current_light_intensity >= SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[SNAI_all_device_value[3]])//光照度报警判断
				{
					//SNAI_DEBUG_INFO("设备:【%u】当前值【%uLux】临界值【%uLux】开启高亮度报警！",SNAI_all_device_value[3],Current_light_intensity,SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[SNAI_all_device_value[3]]);//调试用
					strcpy(SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_NAME[SNAI_DEVICE_EVN_REPOR.Event_Name_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1],"ILLumination_Alarm");
					dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].type = LEDA_TYPE_ENUM;
					dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].type = LEDA_TYPE_INT;
					strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].key ,"Light_illumination");
					sprintf(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].value ,"%s",dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].value);
					strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].key ,"AlarmType");
		        	strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].value ,"1");//0正常/1高量/2低量
		        	SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_TRIGGER[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]] |= 2;//允许上报事件
				}
				record_insert(SNAI_all_device_value[3], SNAI_all_device_value[3], SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]], timestamp,SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_all_device_value[3]],Current_light_intensity,0,1);
				record_update(SNAI_all_device_value[3],SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_all_device_value[3]],Current_light_intensity);
            }
            break;
        case SNAI_TYPE_Position:
			Current_Position_value = (SNAI_all_device_value[7] & 0x7F)*256;
			Current_Position_value += (SNAI_all_device_value[8]>>4)*16+
			            (SNAI_all_device_value[8]&0x0F);
			if(SNAI_all_device_value[3] == 11)
			{
	            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[SNAI_all_device_value[3]] == 1)
	            {
	                dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].type = LEDA_TYPE_INT;
	                strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].key ,"Position_Value");
	                sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].value,"%u",Current_Position_value);
	                //SNAI_DEBUG_INFO("获取【%u#】位置LEFT传感器偏移距离【%umm】",SNAI_all_device_value[3],Current_Position_value);//11# 调试用
	                if(SNAI_Period_Set.SNAI_Main_Filtration_Flag == 0)
	                {
	                	SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]] = 1;
	                }
					else
					{
	                	SNAI_RS485_DATA_Filtration(SNAI_all_device_value[3],&Current_Position_value,0,SNAI_TYPE_Position);
					}
					if(Current_Position_value >= SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[SNAI_all_device_value[3]])//位移长度报警判断
					{
						//SNAI_DEBUG_INFO("设备:【%u】当前值【%umm】临界值【%umm】开启左长位移报警！",SNAI_all_device_value[3],Current_Position_value,SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[SNAI_all_device_value[3]]);//调试用
						strcpy(SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_NAME[SNAI_DEVICE_EVN_REPOR.Event_Name_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1],"Position_Alarm");
						dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].type = LEDA_TYPE_ENUM;
						dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].type = LEDA_TYPE_INT;
						strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].key ,"Position_Value");
						sprintf(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].value ,"%s",dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].value);
						strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].key ,"AlarmType");
			        	strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].value ,"1");//0正常/1高量/2低量
			        	SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_TRIGGER[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]] |= 2;//允许上报事件
					}
            	}
        	}
	        else
			{
	            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[SNAI_all_device_value[3]] == 1)
	            {
	                dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].type = LEDA_TYPE_INT;
	                strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].key ,"Position_Value");
	                sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].value,"%u",Current_Position_value);
	                //SNAI_DEBUG_INFO("获取【%u#】位置RIGHT传感器偏移距离【%umm】",SNAI_all_device_value[3],Current_Position_value);//12#调试用
	                if(SNAI_Period_Set.SNAI_Main_Filtration_Flag == 0)
	                {
	                	SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]] = 1;
	                }
					else
					{
	                	SNAI_RS485_DATA_Filtration(SNAI_all_device_value[3],&Current_Position_value,0,SNAI_TYPE_Position);
					}
					if(Current_Position_value >= SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[SNAI_all_device_value[3]])//位移长度报警判断
					{
						//SNAI_DEBUG_INFO("设备:【%u】当前值【%umm】临界值【%umm】开启右长位移报警！",SNAI_all_device_value[3],Current_Position_value,SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[SNAI_all_device_value[3]]);//调试用
						strcpy(SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_NAME[SNAI_DEVICE_EVN_REPOR.Event_Name_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1],"Position_Alarm");
						dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].type = LEDA_TYPE_ENUM;
						dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].type = LEDA_TYPE_INT;
						strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].key ,"Position_Value");
						sprintf(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].value ,"%s",dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].value);
						strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].key ,"AlarmType");
			        	strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].value ,"1");//0正常/1高量/2低量
			        	SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_TRIGGER[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]] |= 2;//允许上报事件
					}
	            }
				record_insert(SNAI_all_device_value[5], SNAI_all_device_value[3], SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]], timestamp,SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_all_device_value[3]],Current_Position_value,0,1);
				record_update(SNAI_all_device_value[3],SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_all_device_value[3]],Current_Position_value);
			}
            break;
        case SNAI_TYPE_Co:
            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[SNAI_all_device_value[3]] == 1)
            {
                Current_Co_value = (SNAI_all_device_value[7] & 0x7F)*256;
                Current_Co_value += (SNAI_all_device_value[8]>>4)*16+
                                        (SNAI_all_device_value[8]&0x0F);
                dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].type = LEDA_TYPE_INT;
                strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].key ,"GasConcentration");
                sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].value,"%u",Current_Co_value);
                //SNAI_DEBUG_INFO("获取【%u#】一氧化碳值【%uppm】",SNAI_all_device_value[3],Current_Co_value);//13# 调试用
				if(SNAI_Period_Set.SNAI_Main_Filtration_Flag == 0)
				{
                	SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]] = 1;
				}
				else
				{
                	SNAI_RS485_DATA_Filtration(SNAI_all_device_value[3],&Current_Co_value,0,SNAI_TYPE_Co);

                }
				if(Current_Co_value >= SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[SNAI_all_device_value[3]])//气体浓度报警判断
				{
					//SNAI_DEBUG_INFO("设备:【%u】当前值【%uppm】临界值【%uppm】开启高浓度报警！",SNAI_all_device_value[3],Current_Co_value,SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[SNAI_all_device_value[3]]);//调试用
					strcpy(SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_NAME[SNAI_DEVICE_EVN_REPOR.Event_Name_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1],"CO_Alarm");
					dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].type = LEDA_TYPE_ENUM;
					dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].type = LEDA_TYPE_INT;
					strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].key ,"GasConcentration");
					sprintf(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].value ,"%s",dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].value);
					strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].key ,"AlarmType");
		        	strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].value ,"1");//0正常/1高量/2低量
		        	SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_TRIGGER[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]] |= 2;//允许上报事件
				}
				record_insert(SNAI_all_device_value[5], SNAI_all_device_value[3], SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]], timestamp,SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_all_device_value[3]],Current_Co_value,0,1);
				record_update(SNAI_all_device_value[3],SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_all_device_value[3]],Current_Co_value);
            }
            break;

        case SNAI_TYPE_Water_Meter:
                //this is not standrad agreement
            break;
        case SNAI_TYPE_Negative_Pressure:
            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[SNAI_all_device_value[3]] == 1)
            {
                Current_Negative_Pressure_value = (SNAI_all_device_value[7] & 0x7F)*256;
                Current_Negative_Pressure_value += (SNAI_all_device_value[8]>>4)*16+
                                        (SNAI_all_device_value[8]&0x0F);
                dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].type = LEDA_TYPE_INT;
                strcpy(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].key ,"Current_Negative_pressure");
                sprintf(dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].value,"%u",Current_Negative_Pressure_value);
                //SNAI_DEBUG_INFO("获取【%u#】负压值【%uPa】",SNAI_all_device_value[3],Current_Negative_Pressure_value);//7#调试用
				if(SNAI_Period_Set.SNAI_Main_Filtration_Flag == 0)
				{
                	SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]] = 1;

                }
				else
				{
                	SNAI_RS485_DATA_Filtration(SNAI_all_device_value[3],&Current_Negative_Pressure_value,0,SNAI_TYPE_Negative_Pressure);
				}
				if(Current_Negative_Pressure_value >= SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[SNAI_all_device_value[3]])//负压报警判断
				{
					//SNAI_DEBUG_INFO("设备:【%u】当前值【%uPa】临界值【%uPa】开启高负压报警！",SNAI_all_device_value[3],Current_Negative_Pressure_value,SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[SNAI_all_device_value[3]]);//调试用
					strcpy(SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_NAME[SNAI_DEVICE_EVN_REPOR.Event_Name_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1],"Negative_Press_Alarm");
					dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].type = LEDA_TYPE_ENUM;
					dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].type = LEDA_TYPE_INT;
					strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].key ,"Current_Negative_pressure");
					sprintf(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+2].value ,"%s",dev_proper_data[SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]].value);
					strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].key ,"AlarmType");
		        	strcpy(dev_event_data[SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]]+1].value ,"1");//0正常/1高量/2低量
		        	SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_TRIGGER[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]]] |= 2;//允许上报事件
				}
				record_insert(SNAI_all_device_value[5], SNAI_all_device_value[3], SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[SNAI_all_device_value[3]], timestamp,SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_all_device_value[3]],Current_Negative_Pressure_value,0,1);
				record_update(SNAI_all_device_value[3],SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_all_device_value[3]],Current_Negative_Pressure_value);
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
		time_field_update(SNAI_all_device_value[3], timestamp);//时间戳更新
		record_update_state(SNAI_all_device_value[3], 1);
    }
}
/*@Fn 设备数据过滤
* @Par 485地址 / 数据 / 第几类型值温度【0】or湿度【1】
*
*/
bool SNAI_RS485_DATA_Filtration(unsigned char RS485_ADDR,void *data,unsigned char Parameter_n,unsigned char type)
{
    struct tm  *DATA_Filtration_tp;
    time_t DATA_Filtration_t = time(NULL);
    DATA_Filtration_tp = localtime(&DATA_Filtration_t);
    int Current_date_Min  = DATA_Filtration_tp->tm_min;//当前时间
    double Accumulate_value = 0.0;
    float float_tmp = 0.0;
    unsigned short other_data = 0;
		if(type == SNAI_TYPE_Boiler || type == SNAI_TYPE_tmpt || type == SNAI_TYPE_out_tmpt )//按类型匹配
		{
            float_tmp = *((float *)data);
		}
		else if(type == SNAI_TYPE_Water_Meter)
		{
			Accumulate_value = *((double *)data);
		}
		else if(type == SNAI_TYPE_Co2 || type == SNAI_TYPE_NH3 || type == SNAI_TYPE_Illumination || type == SNAI_TYPE_Position || type == SNAI_TYPE_Co || type == SNAI_TYPE_Negative_Pressure || type == SNAI_TYPE_Wind_Direction || type == SNAI_TYPE_Wind_Speed || type == SNAI_TYPE_Other)
		{
			other_data = *((unsigned short *)data);
		}
		else
		{
			SNAI_DEBUG_INFO("错误，无匹配项！");
	        return 1;
		}
		
		if(Parameter_n == 0)
		{
			switch(type)
			{
				case SNAI_TYPE_Boiler:
				case SNAI_TYPE_tmpt:
				case SNAI_TYPE_out_tmpt:	
	    			if(SNAI_ALL_DEVICE_OLD_DATA.SNAI_485dev_OLD_DATA_TMP[RS485_ADDR] == float_tmp && 																					  SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Flag[RS485_ADDR] != 0)//当前值等于上次值，且第二次进入！则不上报数据！否则当前值写入旧数据，作为下次判断依据。
					{
						//SNAI_DEBUG_INFO("温度重复，超时检测中...");//调试用
						Check_Filtration_Timeout(RS485_ADDR);                                                        
					}
					else
					{
                        //SNAI_DEBUG_INFO("地址【%u】温度不重复，Origin【%dmin】",RS485_ADDR,Current_date_Min);//调试用
                        SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Date_Origin_M[RS485_ADDR] = Current_date_Min;//更新最近时间分钟
						SNAI_ALL_DEVICE_OLD_DATA.SNAI_485dev_OLD_DATA_TMP[RS485_ADDR] = float_tmp;//新值入库
                        SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[RS485_ADDR]] = 1;//更新//允许上报数据
                        //SNAI_DEBUG_INFO("地址【%u】本次数据上报【允许】",RS485_ADDR);//调试用
                        //SNAI_DEBUG_INFO("地址【%u】时间录入值【%dmin】",RS485_ADDR,SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Date_Origin_M[RS485_ADDR]);//调试用
					}
					SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Flag[RS485_ADDR] = 1;
					break;
                case SNAI_TYPE_Negative_Pressure:
                case SNAI_TYPE_Co2:
                case SNAI_TYPE_NH3:
                case SNAI_TYPE_Illumination:
                case SNAI_TYPE_Position:
								case SNAI_TYPE_Co:
								case SNAI_TYPE_Wind_Direction:
								case SNAI_TYPE_Wind_Speed:
								case SNAI_TYPE_Other:
                        if(SNAI_ALL_DEVICE_OLD_DATA.SNAI_485dev_OLD_DATA_INT[RS485_ADDR] == other_data && 																					  SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Flag[RS485_ADDR] != 0)//当前值等于上次值，且第二次进入！则不上报数据！否则当前值写入旧数据，作为下次判断依据。
                        {
							//SNAI_DEBUG_INFO("地址【%u】数据重复，超时检测中...",RS485_ADDR);//调试用
                            Check_Filtration_Timeout(RS485_ADDR);
                        }
                        else
                        {
                            //SNAI_DEBUG_INFO("地址【%u】数据不重复，Origin【%dmin】",RS485_ADDR,Current_date_Min);//调试用
                            SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Date_Origin_M[RS485_ADDR] = Current_date_Min;//更新最近时间分钟
                            SNAI_ALL_DEVICE_OLD_DATA.SNAI_485dev_OLD_DATA_INT[RS485_ADDR] = other_data;//新值入库
                            SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[RS485_ADDR]] = 1;//更新//允许上报数据
                            //SNAI_DEBUG_INFO("地址【%u】本次数据上报【允许】",RS485_ADDR);//调试用
                            //SNAI_DEBUG_INFO("地址【%u】时间录入值【%dmin】",RS485_ADDR,SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Date_Origin_M[RS485_ADDR]);//调试用
                        }
                        SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Flag[RS485_ADDR] = 1;
                    break;
                case SNAI_TYPE_Water_Meter:
                        if(SNAI_ALL_DEVICE_OLD_DATA.SNAI_485dev_OLD_DATA_Flow_Rate_17 == Accumulate_value && 																					  SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Flag[RS485_ADDR] != 0)//当前值等于上次值，且第二次进入！则不上报数据！否则当前值写入旧数据，作为下次判断依据。
                        {
							//SNAI_DEBUG_INFO("水表流量数据重复，超时检测中...");//调试用
                            Check_Filtration_Timeout(RS485_ADDR);
                        }
                        else
                        {
                            SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Date_Origin_M[RS485_ADDR] = Current_date_Min;//更新最近时间分钟
                            SNAI_ALL_DEVICE_OLD_DATA.SNAI_485dev_OLD_DATA_Flow_Rate_17 = Accumulate_value;//新值入库
                            SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[RS485_ADDR]] = 1;//更新//允许上报数据
                            //SNAI_DEBUG_INFO("地址【%u】水表流量数据不重复，本次数据上报【允许】",RS485_ADDR);//调试用
                        }
                        SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Flag[RS485_ADDR] = 1;
                    break;
                default:
                        SNAI_DEBUG_INFO("【异常匹配！】");
                    break;
			}
		}
        else//第二个湿度参数或者流速参数的时候
        {
            switch(type)
            {
                case SNAI_TYPE_tmpt:
                case SNAI_TYPE_out_tmpt:
                    if(SNAI_ALL_DEVICE_OLD_DATA.SNAI_485dev_OLD_DATA_Humi[RS485_ADDR] == float_tmp)//当前值等于上次值，则不上报数据！否则当前值写入旧数据，作为下次判断依据。
                    {
                        //温度的超时检测结果
                        //SNAI_DEBUG_INFO("湿度重复，判断温度更新需求...");//调试用
                        //SNAI_DEBUG_INFO("地址【%u】上次值【%u】",RS485_ADDR,SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[RS485_ADDR]]);//调试用
                        if(SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[RS485_ADDR]])
                        {
                            SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[RS485_ADDR]] = 1;//强制更新
                            //SNAI_DEBUG_INFO("【强制更新！】");//调试用
                        }
                        else
                        {
                            //SNAI_DEBUG_INFO("【禁止强制更新！】");//温度也是重复值时禁止更新  //调试用
                        }
                    }
                    else
                    {
                        //SNAI_DEBUG_INFO("地址【%u】湿度不重复，Origin【%dmin】",RS485_ADDR,Current_date_Min);//调试用
                        SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Date_Origin_M[RS485_ADDR] = Current_date_Min;//更新最近时间分钟
                        SNAI_ALL_DEVICE_OLD_DATA.SNAI_485dev_OLD_DATA_Humi[RS485_ADDR] = float_tmp;//新值入库
                        SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[RS485_ADDR]] = 1;//更新//允许上报数据
                        //SNAI_DEBUG_INFO("地址【%u】本次数据上报【允许】",RS485_ADDR);//调试用
                        //SNAI_DEBUG_INFO("地址【%u】时间录入值【%dmin】",RS485_ADDR,SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Date_Origin_M[RS485_ADDR]);//调试用
                    }
                    //SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Flag[RS485_ADDR] = 1;
                break;
                default:
                    SNAI_DEBUG_INFO("【异常匹配！】");
					return 1;
                break;
			}	
		}
	return 0;
}
/*过滤超时检测*/
bool Check_Filtration_Timeout(unsigned char addr)
{
    struct tm  *DATA_Filtration_tp;
    time_t DATA_Filtration_t = time(NULL);
    DATA_Filtration_tp = localtime(&DATA_Filtration_t);
    int Current_date_Min  = DATA_Filtration_tp->tm_min;//当前时间
        if(59 > SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Date_Origin_M[addr]+SNAI_Period_Set.SNAI_485dev_Filtration_Timeout_Period[addr])//以下
		{
            if(Current_date_Min > SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Date_Origin_M[addr]+SNAI_Period_Set.SNAI_485dev_Filtration_Timeout_Period[addr])//超时
			{
            //SNAI_DEBUG_INFO("地址【%u】超时，Origin【%dmin】Current【%dmin】",addr,SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Date_Origin_M[addr],Current_date_Min);//调试用
				SNAI_DEBUG_INFO("超时更新触发！");
						SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[addr]] = 1;//更新
        		SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Date_Origin_M[addr] = Current_date_Min;//更新最近时间分钟       
			}
            else//未达到SNAI_Filtration_Timeout  min超时 //SNAI_Period_Set.SNAI_485dev_Filtration_Timeout_Period[addr]每个各设备独立超时控制
            {
                SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[addr]] = 0;//禁止更新
                //SNAI_DEBUG_INFO("地址【%u】禁止更新触发！【%u】",addr,SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[addr]]);//调试用
            }
        }
        if(59 <= SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Date_Origin_M[addr]+SNAI_Period_Set.SNAI_485dev_Filtration_Timeout_Period[addr])//以上
        {
			int abs_value = abs(59-SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Date_Origin_M[addr]-SNAI_Period_Set.SNAI_485dev_Filtration_Timeout_Period[addr]);
            if(Current_date_Min <= abs_value && Current_date_Min >= abs(abs_value-2))//超时
			{
                //SNAI_DEBUG_INFO("地址【%u】超时，Origin【%dmin】Current【%dmin】",addr,SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Date_Origin_M[addr],Current_date_Min);//调试用
				SNAI_DEBUG_INFO("超时更新触发！");
								SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[addr]] = 1;//更新
                SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Date_Origin_M[addr] = Current_date_Min;//更新最近时间分钟
			}
            else//未达到SNAI_Filtration_Timeout  min超时
            {
                SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[addr]] = 0;//禁止更新
                //SNAI_DEBUG_INFO("地址【%u】禁止更新触发！【%u】",addr,SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[addr]]);//调试用
            }
        }

        return SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[addr]];
}
void* status_report(void* data)
{
    int i = 0;
    int Para_num = 0,temp = 0,Event_Name_num = 0,Event_Name_temp = 0,Event_Name_offset = 0;
		unsigned int Event_Para_offset = 0,Event_Par_temp = 0,Event_Para_count_temp = 0;//偏移--参数数量--末尾地址
    SNAI_circular_buffer *cb = (SNAI_circular_buffer *)(data);
    signal(SIGKILL,SNAI_driver_exit);
    /* 对已上线设备每隔2秒钟上报一次温度数据和事件 */
    while (1)
    {
			//SNAI_DEBUG_INFO("加锁！");//调试用
			pthread_mutex_lock(&SNAI_Decode_mutex_lock);
			//SNAI_DEBUG_INFO("挂起线程！");//调试用
			pthread_cond_wait(&SNAI_cond, &SNAI_Decode_mutex_lock);
			//SNAI_DEBUG_INFO("等待条件变量激活...");//调试用
    	if(SNAI_Gateway_Power_Switch_Flag != 0)
    	{
					//SNAI_DEBUG_INFO("条件变量激活！");//调试用
	        for (i = 0; i < g_dev_handle_count; i++)
	        {
							if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[i]] == 1)
					{
	            if(SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[i] == 1)
	            {
			      /* report device properties */
	              temp = SNAI_ALL_DEVICE_REPORT.Parameter_ptr[i]+SNAI_ALL_DEVICE_REPORT.Parameter_count[i];
					SNAI_DEBUG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
				  SNAI_DEBUG_INFO("设备【%u】【数据】最大偏移至数组下标【%d】",SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[i],temp-1);
	              for(Para_num = SNAI_ALL_DEVICE_REPORT.Parameter_ptr[i];Para_num < temp;Para_num++)
			      {
					if(0 < strlen(dev_proper_data[Para_num].value))
					{
			            SNAI_DEBUG_INFO("上传时设备数值【%s】",dev_proper_data[Para_num].value);
			            SNAI_DEBUG_INFO("上传时设备字符数【%u】",strlen(dev_proper_data[Para_num].value));//交叉编译不同long unsigned
					}
	              }
				 SNAI_DEBUG_INFO("上传数据首地址:【%d】数量:【%d】",SNAI_ALL_DEVICE_REPORT.Parameter_ptr[i],SNAI_ALL_DEVICE_REPORT.Parameter_count[i]);
	             leda_report_properties(g_dev_handle_list[i],dev_proper_data+SNAI_ALL_DEVICE_REPORT.Parameter_ptr[i], SNAI_ALL_DEVICE_REPORT.Parameter_count[i]);
	             
				 //memset(dev_proper_data+SNAI_ALL_DEVICE_REPORT.Parameter_ptr[i],0,sizeof(dev_proper_data[0])*SNAI_ALL_DEVICE_REPORT.Parameter_count[i]);//不可瞎用，导致乱码！
	            
	            /* report device event */
					SNAI_DEBUG_INFO("|------------------设备号【%u】事件参数------------------|",SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[i]);
					Event_Name_temp = SNAI_DEVICE_EVN_REPOR.Event_Name_ptr[i]+SNAI_DEVICE_EVN_REPOR.Event_Name_count[i];//首地址+参数个数=尾地址+1
					for(Event_Name_num = SNAI_DEVICE_EVN_REPOR.Event_Name_ptr[i]; Event_Name_num < Event_Name_temp; Event_Name_num++)//依据事件名个数循环
					{
						if(Event_Name_num == SNAI_DEVICE_EVN_REPOR.Event_Name_ptr[i])//如果第一个事件触发那么参数只有1个
						{
							Event_Name_offset += 1;
							Event_Par_temp = 1;
						}
						else //其他事件参数都为2个
						{
							Event_Name_offset += 2;
							Event_Par_temp = 2;
						}
						SNAI_DEBUG_INFO("设备号【%u】事件名轮寻次数+1",SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[i]);
						if((SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_TRIGGER[i] & 0x01) == 1)//从第一个事件开始检测，决定是否上报
						{
							SNAI_DEBUG_INFO("设备号【%u】事件任务轮进入+1",SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[i]);
							Event_Para_count_temp = SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[i]+Event_Name_offset;//加上事件变化偏移量
							//SNAI_DEBUG_INFO("设备号【%u】原始【%u】当前总偏移至【%u】",SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[i],Event_Para_count_temp-Event_Par_temp,Event_Para_count_temp-1);//调试用
							SNAI_DEBUG_INFO("设备号【%u】事件名【%s】参数数量【%u】",SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[i],SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_NAME[Event_Name_num],Event_Par_temp);
							
							for(Event_Para_offset = (Event_Para_count_temp-Event_Par_temp); Event_Para_offset < Event_Para_count_temp; Event_Para_offset++)//循环调试-->展示数据用
							{
							          SNAI_DEBUG_INFO("事件参数名【%s】事件上传时数值【%s】",dev_event_data[Event_Para_offset].key,dev_event_data[Event_Para_offset].value);
							         
							}
							//leda_report_event(g_dev_handle_list[i], SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_NAME[SNAI_DEVICE_EVN_REPOR.Event_Name_ptr[i]+Event_Name_offset], dev_event_data, Event_Para_offset);//上报名称
							leda_report_event(g_dev_handle_list[i], SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_NAME[Event_Name_num], dev_event_data+(Event_Para_count_temp-Event_Par_temp), Event_Par_temp);//上报名称	
							SNAI_DEBUG_INFO("上传事件首地址:【%d】偏移【%d】总数量:【%d】",Event_Para_count_temp-Event_Par_temp,Event_Par_temp-1,Event_Par_temp);
						}
						SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_TRIGGER[i] >>= 1;//下一个事件检测
				          
					}
					Event_Name_offset = 0;
					SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_TRIGGER[i] = 0;
					SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[i] = 0;
			}
      else
      {
          //SNAI_DEBUG_INFO("*********设备【%u】未准备好数据，不上传！【线程号“%d”】*********",SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[i],i);//调试用
      }
		}
			}
    	}
				//SNAI_DEBUG_INFO("解锁！");//调试用
				pthread_mutex_unlock(&SNAI_Decode_mutex_lock);
				
        if(SNAI_Period_Set.SNAI_Gateway_Report_Period == 0)
		{
			//SNAI_DEBUG_INFO("挂起本线程150ms！");//调试用
			usleep(150000);//最快上报频率
		}
		else
		{
					//SNAI_DEBUG_INFO("挂起本线程%us！",SNAI_Period_Set.SNAI_Gateway_Report_Period);//调试用
        	sleep(SNAI_Period_Set.SNAI_Gateway_Report_Period);//Origin_2s
		}
    }
}

void* TX_READ(void* data)
{
    SNAI_circular_buffer *cb = (SNAI_circular_buffer *) (data);
    unsigned char SNAI_DEVICE_ReadBUFF[8] = {0xFF,0xEE,0x08,0x02,0x52,0x01,0xE2,0xC2};
    static unsigned short times = 1;
	unsigned char  current_device_cunt = 0;
	unsigned char  TX_first_polling_flag = 0;
    unsigned short SNAI_TX_CRC_value = 0;
    unsigned char SNAI_TX_CRC_value_L = 0;
    unsigned char SNAI_TX_CRC_value_H = 0;
    unsigned char SNAI_485_ADDR = 0x01;//start_origin 0x02
    while(1)
    {
		//SNAI_DEBUG_INFO("TX_is Aliver！status【%d】",SNAI_Gateway_Power_Switch_Flag);调试用
    	if(SNAI_Gateway_Power_Switch_Flag != 0 && SNAI_Gateway_Power_Switch_Flag != 2)
    	{
	        
	        if(SNAI_file_id == -1)
	        {
	            usart_init();
	            SNAI_DEBUG_INFO("再次尝试打开！");
	        }
	        else
	        {
				if(TX_first_polling_flag != 0)
				{
					current_device_cunt = 0;
					for(unsigned char dev_addr_c = 0;dev_addr_c < ALL_DEVICE_COUNT+1;dev_addr_c++)//一直判断设备存在数量
					{
							if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[dev_addr_c] == 1)
								current_device_cunt++;
					}
					current_device_cunt -= 1;//去掉虚拟网关设备--
					if(current_device_cunt < 1)current_device_cunt = 1;//最小保证
				}
				else
				{
						SNAI_Period_Set.SNAI_polling_cycle[SNAI_485_ADDR] = SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_485_ADDR]/100;//第一次，立刻轮寻
						TX_first_polling_flag = 1;
				}	
	            if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[SNAI_485_ADDR] == 1)//设备存在，polling this addr
	            {
					//SNAI_DEBUG_INFO("设备【%u】当前计数【%u】>=? 目标计数【%u】设定时【%ums】",SNAI_485_ADDR,SNAI_Period_Set.SNAI_polling_cycle[SNAI_485_ADDR],SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_485_ADDR]/100,SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_485_ADDR]);//调试周期用打印
					
					if(SNAI_Period_Set.SNAI_polling_cycle[SNAI_485_ADDR] >= SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_485_ADDR]/100)
					{
						pthread_mutex_lock(&SNAI_GET_Properties_mutex_lock);
						SNAI_Period_Set.SNAI_polling_cycle[SNAI_485_ADDR] = 1;
		                //SNAI_DEBUG_INFO("串口发送%u次",times);//调试用
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
		                SNAI_TX_CRC_value = CRC_Return(SNAI_DEVICE_ReadBUFF,6);
		                SNAI_TX_CRC_value_L =(unsigned char)(SNAI_TX_CRC_value &0x00FF);//有无符号重要！
		                SNAI_TX_CRC_value_H = (unsigned char)((SNAI_TX_CRC_value>>8)&0x00FF);
		                SNAI_DEVICE_ReadBUFF[6] = SNAI_TX_CRC_value_L;
		                SNAI_DEVICE_ReadBUFF[7] = SNAI_TX_CRC_value_H;
		                usart_tx(SNAI_DEVICE_ReadBUFF ,8);
		                for(int TX_data = 0;TX_data < 8;TX_data++)
		                {
		                   //SNAI_DEBUG_INFO("发送数据:%02X",SNAI_DEVICE_ReadBUFF[TX_data]);//调试用
		                }
										pthread_mutex_unlock(&SNAI_GET_Properties_mutex_lock);
	                }
					usleep(100000);//轮寻基时100ms
					SNAI_Period_Set.SNAI_polling_cycle[SNAI_485_ADDR] += current_device_cunt;//设备赋值轮寻一圈值，上方if使用时说明已轮寻一圈
	            }
	  	    SNAI_485_ADDR++;
		    SNAI_485_ADDR = (SNAI_485_ADDR > ALL_DEVICE_COUNT?0x01:SNAI_485_ADDR);	
	        }  
    	}
			usleep(1000);//统一基时1000us
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
    const char *get_dat[]={
		"CurrentTemperature",
		"CurrentHumidity",
		"Current_Negative_pressure",
		"Position_Value",
		"Light_illumination",
		"GasConcentration",
		"CurrentWater_Yield",
		"CurrentFlow_Rate"
						};
    unsigned char  SNAI_GET_DEVICE_ReadBUFF[8] = {0xFF,0xEE,0x08,0x02,0x52,0x01,0xE2,0xC2};
    unsigned short SNAI_GET_CRC_value   = 0;
    unsigned char  SNAI_GET_CRC_value_L = 0;
    unsigned char  SNAI_GET_CRC_value_H = 0;
    int i = 0,Key_num = 0;
    SNAI_GET_DEVICE_ReadBUFF[3] = SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle];//获取当前线程设备485地址
    for (i = 0; i < properties_count; i++)
    {
        log_i(LED_TAG_NAME, "get_property %s: ", properties[i].key);
        SNAI_DEBUG_INFO("获取设备属性来自云端identification【%s】",properties[i].key);
        for(Key_num = 0; Key_num < 8;Key_num++)
        {
            if (!strcmp(properties[i].key, get_dat[Key_num]))//比较 属性名 字符串是否一致，返回值为0则一致
            {
                SNAI_DEBUG_INFO("获取设备属性来自本地->485地址【%u】【%s】",SNAI_GET_DEVICE_ReadBUFF[3],get_dat[Key_num]);
                SNAI_GET_CRC_value = CRC_Return(SNAI_GET_DEVICE_ReadBUFF,6);
                SNAI_GET_CRC_value_L = (unsigned char)(SNAI_GET_CRC_value &0x00FF);//有无符号重要！
                SNAI_GET_CRC_value_H = (unsigned char)((SNAI_GET_CRC_value>>8)&0x00FF);
                SNAI_GET_DEVICE_ReadBUFF[6] = SNAI_GET_CRC_value_L;
                SNAI_GET_DEVICE_ReadBUFF[7] = SNAI_GET_CRC_value_H;
                pthread_mutex_lock(&SNAI_GET_Properties_mutex_lock);
                usart_tx(SNAI_GET_DEVICE_ReadBUFF ,8);
                for(int GET_data = 0;GET_data < 8;GET_data++)
                {
                    SNAI_DEBUG_INFO("立即发送数据:%02X",SNAI_GET_DEVICE_ReadBUFF[GET_data]);
                }
                pthread_mutex_unlock(&SNAI_GET_Properties_mutex_lock);
                usleep(100000);//100ms
                /* 作为演示，填写获取属性数据为模拟数据 */
                properties[i].type = LEDA_TYPE_DOUBLE;//
                sprintf(properties[i].value, "31.0");
                log_i(LED_TAG_NAME, "%s\r\n",  properties[i].value);
                break;//匹配下一个属性标识
            }
            usleep(100000);//100ms
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
	const char *Service_dat1[]= {
	"Power_Switch",
	"Gateway_Power_Switch",
	"Gateway_Setting_Period",
	"Setting_Period",
	"Env_Control",
	"Env_Control_Humi",
	"Over_Timeout",
	"Switch_Filtration",
	"Manul"
								    };
	const char *Service_dat2[]= {
	"Power_Switch",
	"Period_Time",
	"Alarm_Temp",
	"Alarm_Humi",
	"Alarm_Boiler",
	"Alarm_Negative_pressure",
	"Alarm_Co2",
	"Alarm_NH3",
	"Alarm_iLLumination",
	"Alarm_Position_Value",
	"Alarm_Water",
	"Alarm_Co",
	"Over_Timeout",
	"Main_Switch_Filtration",
	"Manul"
												};
    int i = 0;
		SNAI_Config *cnf = SNAI_cnf_read_config("/linkedge/run/SNAI_cnf.ini", '#', '=');
		char config_dev_n[12];								
		sprintf(config_dev_n,"485_DEV_%u",SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle]);
    /* service_name为该驱动物模型自定义方法名称 */
    log_i(LED_TAG_NAME, "service_name: %s\r\n", service_name);
    SNAI_DEBUG_INFO("service_name:【%s】",service_name);
    /* 获取service_name方法的参数名称和值信息 */
    for (i = 0; i < data_count; i++)
    {
        log_i(LED_TAG_NAME, "input_data %s: %s\r\n", data[i].key, data[i].value);
        SNAI_DEBUG_INFO("input_data:【%s】【%s】",data[i].key,data[i].value);
		for(int Service_N_c = 0;Service_N_c < 9;Service_N_c++)
			{
				if (!strcmp(service_name, Service_dat1[Service_N_c]))//匹配到服务名称
        		{
					for (int Service_K_c = 0;Service_K_c < 15;Service_K_c++)
					{
						if (!strcmp(data[i].key, Service_dat2[Service_K_c]))//匹配到输入参数名称
        				{
							if(Service_K_c == 0)
							{
								if(Service_N_c == 0)
								{
									SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle]] = atoi(data[i].value);//停止轮询
									SNAI_cnf_add_option(cnf, config_dev_n, Service_dat2[Service_K_c], data[i].value);									
									SNAI_DEBUG_INFO("设备:【%u】开关状态:【%d】",SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle],SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle]]);
								}
								if(Service_N_c == 1)
								{
									SNAI_Gateway_Power_Switch_Flag = atoi(data[i].value);//为0 /停止轮询，停止上报，停止解析一切数据 //=2停止主动轮寻
									SNAI_cnf_add_option(cnf, "485_Gateway", Service_dat2[Service_K_c], data[i].value);
									SNAI_DEBUG_INFO("网关开关状态:【%d】",SNAI_Gateway_Power_Switch_Flag);
								}
								if(SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle]] == 0)record_delete(SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle]);
								
							}
							if(Service_K_c == 1)
							{
								if (Service_N_c == 2)//网关的周期/s
								{
									sscanf(data[i].value,"%u",&SNAI_Period_Set.SNAI_Gateway_Report_Period);
									SNAI_cnf_add_option(cnf, "485_Gateway", Service_dat2[Service_K_c], data[i].value);
									SNAI_DEBUG_INFO("网关上报周期:【%uS】",SNAI_Period_Set.SNAI_Gateway_Report_Period);
								}
								if (Service_N_c == 3)//设备轮询周期/*1000=us
								{
									sscanf(data[i].value,"%u",&SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle]]);
									SNAI_cnf_add_option(cnf, config_dev_n, Service_dat2[Service_K_c], data[i].value);
									SNAI_DEBUG_INFO("设备:【%u】轮询周期:【%ums】",SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle],SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle]]);
								}
								record_update(SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle],SNAI_Period_Set.SNAI_485dev_Polling_Period[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle]],0);
							}
							if(Service_K_c == 2 || Service_K_c == 3 || Service_K_c == 4)//报警值
							{
								if(Service_N_c == 5)//湿度报警值
								{
									SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_Humi[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle]] = atof(data[i].value);//485地址区分 -每个地址报警值
									SNAI_cnf_add_option(cnf, config_dev_n, Service_dat2[Service_K_c], data[i].value);
									SNAI_DEBUG_INFO("设备:【%u】湿度报警设定:【%.1f%%】",SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle],SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_Humi[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle]]);
								}
								else
								{
								SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_TMP[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle]] = atof(data[i].value);//485地址区分 -每个地址报警值
								SNAI_cnf_add_option(cnf, config_dev_n, Service_dat2[Service_K_c], data[i].value);
								SNAI_DEBUG_INFO("设备:【%u】温度报警设定:【%.1f℃】",SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle],SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_TMP[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle]]);
								}							
							}
							if(Service_K_c == 5 || Service_K_c == 6 || Service_K_c == 7 || Service_K_c == 8 || Service_K_c == 9 || Service_K_c == 11)//报警值
							{
								sscanf(data[i].value,"%u",&SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle]]);
								if(SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle] == 11)
								{
									SNAI_cnf_add_option(cnf, config_dev_n, "Alarm_Position_Value_L", data[i].value);
								}
								else if(SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle] == 12)
								{
									SNAI_cnf_add_option(cnf, config_dev_n, "Alarm_Position_Value_R", data[i].value);
								}
								else
								{
									SNAI_cnf_add_option(cnf, config_dev_n, Service_dat2[Service_K_c], data[i].value);
								}
								SNAI_DEBUG_INFO("设备:【%u】报警设定:【%u】",SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle],SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle]]);
							}
							if(Service_K_c == 10)
							{
								SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_Water = atof(data[i].value);//485地址区分 -水表报警值
								SNAI_cnf_add_option(cnf, config_dev_n, Service_dat2[Service_K_c], data[i].value);
								SNAI_DEBUG_INFO("设备:【%u】水量报警设定:【%.2f】",SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle],SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_Water);
							}

							if(Service_K_c == 12)
							{
								SNAI_Period_Set.SNAI_485dev_Filtration_Timeout_Period[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle]] = atoi(data[i].value);//每个设备的过滤超时时间设置
								SNAI_cnf_add_option(cnf, config_dev_n, Service_dat2[Service_K_c], data[i].value);
								SNAI_DEBUG_INFO("设备:【%u】过滤超时设定:【%dmin】",SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle],SNAI_Period_Set.SNAI_485dev_Filtration_Timeout_Period[SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[dev_handle]]);
							}
							if(Service_K_c == 13)
							{
								SNAI_Period_Set.SNAI_Main_Filtration_Flag = atoi(data[i].value);//启用网关过滤
								SNAI_cnf_add_option(cnf, "485_Gateway", Service_dat2[Service_K_c], data[i].value);
								SNAI_DEBUG_INFO("网关过滤启用状态:【%d】",SNAI_Period_Set.SNAI_Main_Filtration_Flag);
							}
							if(Service_K_c == 14)
							{
								int temp = 0;
								unsigned char temp_value[16] = {0};
								char temp_ch[1024] = {0};
								strcpy(temp_ch,data[i].value);
								SNAI_DEBUG_INFO("源数据:【%s】",temp_ch);
								if(SNAI_strip_comments(temp_ch, '#'))//去掉空格
								{
									SNAI_DEBUG_INFO("修剪后数据:【%s】",data[i].value);
									if ('[' == data[i].value[0] && ']' == data[i].value[strlen(data[i].value)-1]) {
										for(int count = 1; count < strlen(data[i].value)-1;count++)
											{
												if(data[i].value[count] == ',' || data[i].value[count] == 0x20)continue;
												temp_value[temp] = SNAI_hexCharToValue(data[i].value[count]);
												temp_value[temp] <<= 4;//MSB
												temp_value[temp] += SNAI_hexCharToValue(data[i].value[count+1]);
												count += 1;
												SNAI_DEBUG_INFO("接收自定义数据:【%02X】",temp_value[temp]);
												temp++;//数组下标
											}
									}
								}
								usart_tx(temp_value ,temp+1);//send;					
							}
							break;
						}
					}
				}
		}
    }
		SNAI_cnf_write_file(cnf, "/linkedge/run/SNAI_cnf.ini", "SNAI 485 DEVICE DRIVER CONFIG INFO"); // 将对象写入/linkedge/run/SNAI_cnf.ini文件
		SNAI_delete_config(cnf);
		free(cnf);
		cnf = NULL;
		SNAI_DEBUG_INFO("写入完成！");//调试用
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
/*Preset 每个设备线程号&数据存储首地址,支持24个设备*/
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
	static int DEVICE_EVN_PAR_Ptr = 0;
	static int DEVICE_EVN_NAE_Ptr = 0;
    SNAI_DEBUG_INFO("本次上报参数存入首地址【dev_proper_data[%d]】",DEVICE_PAR_Ptr);
		SNAI_DEBUG_INFO("本次上报事件存入首地址【dev_event_data[%d]】",DEVICE_EVN_PAR_Ptr);
		char* config_dev_n = (char*)malloc(sizeof(char)*12);
		if(NULL == config_dev_n)
		{
		    SNAI_DEBUG_INFO("配置设备名内存分配失败！");
		    return;
		}
		SNAI_Config *cnf = SNAI_cnf_read_config("/linkedge/run/SNAI_cnf.ini", '#', '=');
		if (NULL == cnf) {
			return; /* 创建对象失败 */
		}
		SNAI_cnf_get_value(cnf, "CONFIG_FLAG", "CONFIG_ENABLE");
		int CONFIG_ENABLE = cnf->re_int;
    for(int i =0;i<ALL_DEVICE_COUNT+1;i++)
    {
        if (!strcmp(RS485_ADDR, dat[i]))
        {
						sprintf(config_dev_n,"485_DEV_%u",i);
            if(CONFIG_ENABLE == 1)
						{
							SNAI_cnf_get_value(cnf, config_dev_n, "Power_Switch");
							SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[i] = cnf->re_int;
							SNAI_DEBUG_INFO("读取设备存在否【%d】！",cnf->re_int);//调试用
						}
						else
						{
							SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[i] = 1;
							SNAI_cnf_add_option(cnf, config_dev_n, "Power_Switch", "1");	//写入配置文件
							SNAI_DEBUG_INFO("初次写入配置【%s】Power_Switch【1】！",config_dev_n);//调试用
						}
            SNAI_ALL_DEVICE_REPORT.SNAI_485dev_handle[i] = SNAI_handle;//解析数据读取此处线程号，作为ptr数据存储下标
						SNAI_DEBUG_INFO("当前线程号作为下标【%d】",SNAI_handle);
            SNAI_ALL_DEVICE_REPORT.Parameter_ptr[SNAI_handle] = DEVICE_PAR_Ptr;//线程号对应上报数据 存放地址
            SNAI_DEVICE_EVN_REPOR.Event_Parameter_ptr[SNAI_handle] = DEVICE_EVN_PAR_Ptr;//事件数据存放地址
            SNAI_DEVICE_EVN_REPOR.Event_Name_ptr[SNAI_handle] = DEVICE_EVN_NAE_Ptr;//事件名存放地址
						SNAI_ALL_DEVICE_REPORT.SNAI_485dev_ADDR[SNAI_handle] = i;//数组线程号下标对应设备485地址
						SNAI_ALL_DEVICE_REPORT.SNAI_device_ready[SNAI_handle] = 0;//初始化为0，未准备好数据
            if(i == 2 || i == 3 || i == 4 || i == 5  || i == 17)//定义具备两个参数
            {
                SNAI_ALL_DEVICE_REPORT.Parameter_count[SNAI_handle] = 2;//参数个数
                DEVICE_PAR_Ptr += 2;//下次存放地址offset
            }
            else
            {
                SNAI_ALL_DEVICE_REPORT.Parameter_count[SNAI_handle] = 1;//参数个数
                DEVICE_PAR_Ptr += 1;//下次存放地址offset
            }
			if(i == 2 || i == 3 || i == 4 || i == 5 )//定义具备5个事件参数
			{
				
				SNAI_DEVICE_EVN_REPOR.Event_Parameter_count[SNAI_handle] = 5;//参数个数
				SNAI_DEVICE_EVN_REPOR.Event_Name_count[SNAI_handle] = 3;//事件名个数3
				DEVICE_EVN_PAR_Ptr += 5;//温度及报警/湿度及报警/状态好坏
				DEVICE_EVN_NAE_Ptr += 3;//故障/温度报警/湿度报警	
			}
			else
			{
				SNAI_DEVICE_EVN_REPOR.Event_Parameter_count[SNAI_handle] = 3;//参数个数
				SNAI_DEVICE_EVN_REPOR.Event_Name_count[SNAI_handle] = 2;//事件名个数2
				DEVICE_EVN_PAR_Ptr += 3;//温度及报警/状态好坏
				DEVICE_EVN_NAE_Ptr += 2;//故障/报警
			}
            break;
        }
    }
		SNAI_cnf_write_file(cnf, "/linkedge/run/SNAI_cnf.ini", "SNAI 485 DEVICE DRIVER CONFIG INFO"); // 将对象写入/linkedge/run/SNAI_cnf.ini文件
		SNAI_DEBUG_INFO("写入完成！");//调试用
		//SNAI_print_config(cnf);//调试用
		SNAI_delete_config(cnf);
		free(config_dev_n);
		free(cnf);
		cnf = NULL;
		config_dev_n = NULL;
    SNAI_DEBUG_INFO("下次上报参数存入首地址---【dev_proper_data[%d]】",DEVICE_PAR_Ptr);
	SNAI_DEBUG_INFO("下次上报事件名存入首地址【SNAI_DEVICE_EVN_REPOR.SNAI_DEVICE_EVENT_NAME[%d]】",DEVICE_EVN_NAE_Ptr);
	SNAI_DEBUG_INFO("下次上报事件存入首地址--【dev_event_data[%d]】",DEVICE_EVN_PAR_Ptr);
}
int main(int argc, char** argv)
{
	int    ret         = LE_SUCCESS;
    char*  module_name = NULL;
	char* config_dev_n = NULL;
	int CONFIG_ENABLE = 0;
/*  const char *config_dat[]={
"Main_Switch_Filtration",
"Alarm_1",
"Alarm_Temp",
"Alarm_Humi",
"Alarm_Temp",
"Alarm_Temp",
"Alarm_Boiler",
"Alarm_Negative_pressure",
"Alarm_Co2",
"Alarm_NH3",
"Alarm_iLLumination",
"Alarm_Position_Value_L",
"Alarm_Position_Value_R",
"Alarm_Co",
"Alarm_14",
"Alarm_15",
"Alarm_16",
"Alarm_Water",
"Alarm_18",
"Alarm_19",
"Alarm_20",
"Alarm_21",
"Alarm_22",
"Alarm_23",
"Alarm_24"
    };*/
	filedb_create();
	SNAI_log_init();
    log_init(LED_TAG_NAME, LOG_STDOUT, LOG_LEVEL_DEBUG, LOG_MOD_BRIEF);
	SNAI_DEBUG_INFO("Driver Startup！");
    log_i(LED_TAG_NAME, "demo startup\r\n");
	SNAI_Config *cnf = SNAI_cnf_read_config("/linkedge/run/SNAI_cnf.ini", '#', '=');
	if (NULL == cnf) {
	  return -1; /* 创建对象失败 */
	}
	const char* config_led_ver = "V1.0_20190801";
	SNAI_DEBUG_INFO("MAIN读取当前版本【%s】！",config_led_ver);
	SNAI_cnf_get_value(cnf, "CONFIG_FLAG", "CONFIG_ENABLE");
	CONFIG_ENABLE = cnf->re_int;
	SNAI_DEBUG_INFO("MAIN读取CONFIG_ENBALE【%d】！",cnf->re_int);
	if(CONFIG_ENABLE == 0)
	{
		SNAI_DEBUG_INFO("<初始配置文件>，添加版本号【%s】！",config_led_ver);
		SNAI_cnf_add_option(cnf, "CONFIG_FLAG", "SNAI_LED_Ver",config_led_ver);
	}
	else
	{
		SNAI_cnf_get_value(cnf, "CONFIG_FLAG", "SNAI_LED_Ver");
		if(0 != strcmp(config_led_ver,cnf->re_string))
		{
			SNAI_DEBUG_INFO("<获得新驱动>，更改版本号【%s】>>>【%s】重置设备开关！",cnf->re_string,config_led_ver);
			SNAI_cnf_add_option(cnf, "CONFIG_FLAG", "CONFIG_ENABLE", "0");
			SNAI_cnf_add_option(cnf, "CONFIG_FLAG", "SNAI_LED_Ver",config_led_ver);
			SNAI_cnf_add_option(cnf, "485_Gateway", "Power_Switch", "1");
			SNAI_cnf_add_option(cnf, "485_Gateway", "Main_Switch_Filtration", "1");
			SNAI_cnf_add_option(cnf, "485_Gateway", "Period_Time", "1");
			SNAI_cnf_add_option(cnf, "485_Gateway", "Over_Timeout", "30");	
		}
	}
	SNAI_cnf_write_file(cnf, "/linkedge/run/SNAI_cnf.ini", "SNAI 485 DEVICE DRIVER CONFIG INFO"); // 将对象写入/linkedge/run/SNAI_cnf.ini文件
	SNAI_cnf_get_value(cnf, "485_Gateway", "Power_Switch");
	SNAI_Gateway_Power_Switch_Flag = cnf->re_int;
	SNAI_DEBUG_INFO("初始化读取配置【485_Gateway】设备开关【%d】！", cnf->re_int);//调试用
  //printf("SNAI_cnf_get_value:%s,%d,%d,%f\n",cnf->re_string,cnf->re_int,cnf->re_bool,cnf->re_double);
	SNAI_cnf_get_value(cnf, "485_Gateway", "Main_Switch_Filtration");
	SNAI_Period_Set.SNAI_Main_Filtration_Flag = cnf->re_int;
	SNAI_cnf_get_value(cnf, "485_Gateway", "Period_Time");
	SNAI_Period_Set.SNAI_Gateway_Report_Period = cnf->re_int;
	SNAI_cnf_get_value(cnf, "485_Gateway", "Over_Timeout");
	SNAI_Period_Set.SNAI_485dev_Filtration_Timeout_Period[0] = cnf->re_int;

	config_dev_n = (char*)malloc(sizeof(char)*12);
	if(NULL == config_dev_n)
  {
      SNAI_DEBUG_INFO("配置设备名内存分配失败！");
      return LE_ERROR_UNKNOWN;
  }
	for(int snaist = 0;snaist < ALL_DEVICE_COUNT+1;snaist++)
	{
			SNAI_Period_Set.SNAI_polling_cycle[snaist] = 1;
			SNAI_ALL_DEVICE_REPORT.SNAI_485dev_Data_Filtration_Flag[snaist] = 0;//过滤首次进入标志
	}
	for(int i = 1;i < ALL_DEVICE_COUNT+1;i++)//读取配置
	{						
			sprintf(config_dev_n,"485_DEV_%u",i);
			SNAI_cnf_get_value(cnf, config_dev_n, "Power_Switch");
			SNAI_ALL_DEVICE_REPORT.SNAI_DEVICE_EXIST[i] = cnf->re_int;
			SNAI_cnf_get_value(cnf, config_dev_n, "Period_Time");
			SNAI_Period_Set.SNAI_485dev_Polling_Period[i] = cnf->re_int;
			SNAI_cnf_get_value(cnf, config_dev_n, "Over_Timeout");
			SNAI_Period_Set.SNAI_485dev_Filtration_Timeout_Period[i] = cnf->re_int;
			if(i == 2 || i == 3 || i == 4 || i == 5 )
			{
				SNAI_cnf_get_value(cnf, config_dev_n, "Alarm_Temp");
				SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_TMP[i] = cnf->re_double;
				SNAI_cnf_get_value(cnf, config_dev_n, "Alarm_Humi");
				SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_Humi[i] = cnf->re_double;
			}
			if(i == 6)
			{
				SNAI_cnf_get_value(cnf, config_dev_n, "Alarm_Boiler");
				SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_TMP[i] = cnf->re_double;
			}
			if(i == 7)
			{
				SNAI_cnf_get_value(cnf, config_dev_n, "Alarm_Negative_pressure");
				SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[i] = cnf->re_int;
			}
			if(i == 8)
			{
				SNAI_cnf_get_value(cnf, config_dev_n, "Alarm_Co2");
				SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[i] = cnf->re_int;
			}
			if(i == 9)
			{
				SNAI_cnf_get_value(cnf, config_dev_n, "Alarm_NH3");
				SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[i] = cnf->re_int;
			}
			if(i == 10)
			{
				SNAI_cnf_get_value(cnf, config_dev_n, "Alarm_iLLumination");
				SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[i] = cnf->re_int;
			}
			if(i == 11)
			{
				SNAI_cnf_get_value(cnf, config_dev_n, "Alarm_Position_Value_L");
				SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[i] = cnf->re_int;
			}
			if(i == 12)
			{
				SNAI_cnf_get_value(cnf, config_dev_n, "Alarm_Position_Value_R");
				SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[i] = cnf->re_int;
			}
			if(i == 13)
			{
				SNAI_cnf_get_value(cnf, config_dev_n, "Alarm_Co");
				SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[i] = cnf->re_int;
			}
			if(i == 14)
			{
				SNAI_cnf_get_value(cnf, config_dev_n, "Alarm_14");
				SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[i] = cnf->re_int;
			}
			if(i == 15)
			{
				SNAI_cnf_get_value(cnf, config_dev_n, "Alarm_15");
				SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[i] = cnf->re_int;
			}
			if(i == 16)
			{
				SNAI_cnf_get_value(cnf, config_dev_n, "Alarm_16");
				SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[i] = cnf->re_int;
			}
			if(i == 17)
			{
				SNAI_cnf_get_value(cnf, config_dev_n, "Alarm_Water");
				SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[i] = cnf->re_double;
			}
			if(i == 18)
			{
				SNAI_cnf_get_value(cnf, config_dev_n, "Alarm_18");
				SNAI_Alarm_value.SNAI_485dev_Alarm_DATA_INT[i] = cnf->re_int;
			}
	}
	SNAI_delete_config(cnf);
	free(cnf);
	free(config_dev_n);
	config_dev_n = NULL;
	cnf = NULL;
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
		SNAI_DEBUG_INFO("模块初始化失败退出！！");
        log_e(LED_TAG_NAME, "leda_init failed\r\n");
        return ret;
    }
		SNAI_DEBUG_INFO("解析配置-->");
    /* 解析配置 */
    if (LE_SUCCESS != (ret = get_and_parse_deviceconfig(module_name)))
    {
		SNAI_DEBUG_INFO("解析配置失败！！");
        log_e(LED_TAG_NAME, "parse device config failed\r\n");
        return ret;
    }
	/////////////////////////////////////////////////////////////////////////
	cnf = SNAI_cnf_read_config("/linkedge/run/SNAI_cnf.ini", '#', '=');//读取最新配置文件，更新一次链表信息
	SNAI_cnf_get_value(cnf, "CONFIG_FLAG", "CONFIG_ENABLE");
	CONFIG_ENABLE = cnf->re_int;
	SNAI_DEBUG_INFO("MAIN读取CONFIG_ENBALE【%d】！",cnf->re_int);
	if(CONFIG_ENABLE == 0)
	{
		SNAI_DEBUG_INFO("<初始配置文件>，故本次将其置【1】！");
		SNAI_cnf_add_option(cnf, "CONFIG_FLAG", "CONFIG_ENABLE", "1");
	}
	SNAI_cnf_write_file(cnf, "/linkedge/run/SNAI_cnf.ini", "SNAI 485 DEVICE DRIVER CONFIG INFO"); // 将对象写入/linkedge/run/SNAI_cnf.ini文件
	SNAI_DEBUG_INFO("MAIN读取配置初始化完成！");//调试用
	SNAI_delete_config(cnf);
	free(cnf);
	cnf = NULL;
	SNAI_DEBUG_INFO("建立环形缓冲区-->");
	/* 8K rx cache for usart rx*/
    SNAI_circular_buffer* cb_usart_rx = cb_create(13);//建立环形缓冲区
	SNAI_pthread_opt_user pthread_user_seq[] =//声明.结构体数组，4个元素，每个元素依次：函数指针、void型指针变量、线程id
	{
		{TX_READ, cb_usart_rx},//元素1，数组【0】，因每个元素是结构体所以具有大括号{}
		{usart_rx_start, cb_usart_rx},
		{status_minitor,NULL},//数组【2】狀態檢測
		{tlv_decode_start,cb_usart_rx},//数组【2】解碼
		{status_report,cb_usart_rx},//设备数据上报
		{NULL ,NULL}//数组【3】
	};
        
	opt_seq_ptr = pthread_user_seq;//opt_seq_ptr指向pthread_user_seq，函数队列usart_rx_start、tlv_decode_start、status_minitor
/*
 * 		SIGINT   2    采用ctrl+c产生该信号
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
    	/* 退出驱动 */
    	
    	leda_exit();
	return LE_SUCCESS;
}

#ifdef __cplusplus
}
#endif
